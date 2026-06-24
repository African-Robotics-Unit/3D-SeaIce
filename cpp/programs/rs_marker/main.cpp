#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <fstream>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static void print_usage(const char* prog)
{
    std::cout << "Usage: " << prog << " [-o output.bag] [-d duration_seconds]\n"
              << "  -o, --output    Record to .bag file     (default: no recording)\n"
              << "  -d, --duration  Duration in seconds     (default: run until q/Ctrl+C)\n"
              << "  -h, --help      Show this message\n";
}

// ── Depth post-processing ─────────────────────────────────────────────────────
struct DepthFilters {
    rs2::decimation_filter   decimation;
    rs2::disparity_transform to_disparity{true};
    rs2::spatial_filter      spatial;
    rs2::temporal_filter     temporal;
    rs2::disparity_transform to_depth{false};
    rs2::hole_filling_filter hole_fill;

    rs2::depth_frame process(rs2::depth_frame depth)
    {
        depth = decimation.process(depth);
        depth = to_disparity.process(depth);
        depth = spatial.process(depth);
        depth = temporal.process(depth);
        depth = to_depth.process(depth);
        depth = hole_fill.process(depth);
        return depth.as<rs2::depth_frame>();
    }
};

// ── Pipeline helpers ──────────────────────────────────────────────────────────
struct PipelineCtx {
    rs2::pipeline         pipe;
    rs2::pipeline_profile profile;
};

static PipelineCtx start_pipeline(const std::string& output_file)
{
    PipelineCtx ctx;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16,        5);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 848, 480, RS2_FORMAT_RGB8,       5);
    cfg.enable_stream(RS2_STREAM_ACCEL,             RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO,              RS2_FORMAT_MOTION_XYZ32F);
    if (!output_file.empty())
        cfg.enable_record_to_file(output_file);
    ctx.profile = ctx.pipe.start(cfg);

    auto depth_sensor = ctx.profile.get_device().first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
        depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET,
                                RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
    return ctx;
}

// RS2 colour frame → cloned BGR Mat (safe after frame goes out of scope)
static cv::Mat to_bgr(const rs2::video_frame& f)
{
    cv::Mat m(f.get_height(), f.get_width(), CV_8UC3,
              (void*)f.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat out;
    cv::cvtColor(m, out, cv::COLOR_RGB2BGR);
    return out;
}

// Colourised depth frame → cloned BGR Mat
static cv::Mat depth_to_mat(const rs2::frame& colorized)
{
    auto vf = colorized.as<rs2::video_frame>();
    return cv::Mat(vf.get_height(), vf.get_width(), CV_8UC3,
                   (void*)colorized.get_data(), cv::Mat::AUTO_STEP).clone();
}

static void show_frames(const cv::Mat& color, const cv::Mat& depth)
{
    cv::imshow("RGB + markers",    color);
    cv::imshow("Depth (filtered)", depth);
}

static void print_stats(long long frames, double elapsed,
                        double depth_avg, rs2_vector accel, rs2_vector gyro)
{
    std::cout << "\r"
              << "frames: "   << std::setw(6) << frames
              << "  fps: "    << std::fixed << std::setprecision(1)
                              << std::setw(5) << frames / elapsed
              << "  depth: "  << std::setprecision(3) << std::setw(6) << depth_avg << "m"
              << "  accel: [" << std::setprecision(2)
                              << accel.x << "," << accel.y << "," << accel.z << "]"
              << "  gyro: ["  << gyro.x  << "," << gyro.y  << "," << gyro.z  << "]"
              << "     " << std::flush;
}

// ── Detection result type ─────────────────────────────────────────────────────
struct Detection {
    int                      id;
    std::vector<cv::Point2f> corners;
    cv::Point2f              centre;
};

// ── CSV logging ───────────────────────────────────────────────────────────────
// Derives csv path from bag name: "capture.bag" → "capture_detections.csv"
// Falls back to "detections.csv" when not recording.
static std::string csv_path(const std::string& bag)
{
    if (bag.empty()) return "detections.csv";
    auto dot = bag.rfind('.');
    return (dot == std::string::npos ? bag : bag.substr(0, dot)) + "_detections.csv";
}

static std::ofstream open_csv(const std::string& path)
{
    std::ofstream f(path);
    f << "timestamp_ms,frame,marker_id,cx,cy,depth_m,"
      << "c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y\n";
    return f;
}

static void log_detections(std::ofstream& f, long long frame, double timestamp_ms,
                            const std::vector<Detection>& detections,
                            const rs2::depth_frame& depth)
{
    for (const auto& d : detections) {
        float depth_m = depth.get_distance(
            std::max(0, std::min((int)d.centre.x, depth.get_width()  - 1)),
            std::max(0, std::min((int)d.centre.y, depth.get_height() - 1)));
        f << std::fixed << std::setprecision(3)
          << timestamp_ms    << ","
          << frame           << ","
          << d.id            << ","
          << d.centre.x      << "," << d.centre.y << ","
          << depth_m         << ",";
        for (int i = 0; i < 4; ++i)
            f << d.corners[i].x << "," << d.corners[i].y
              << (i < 3 ? "," : "\n");
    }
    f.flush();
}

// ═════════════════════════════════════════════════════════════════════════════
//  MARKER DETECTION  — edit this section to experiment with AprilTag logic
// ═════════════════════════════════════════════════════════════════════════════

static const std::vector<int> TARGET_IDS = {92, 93, 94, 95};

// Returns detections for TARGET_IDS only, ignoring any other tags in frame.
static std::vector<Detection> detect_markers(const cv::Mat& gray)
{
    static auto dictionary = cv::aruco::getPredefinedDictionary(
                                 cv::aruco::DICT_APRILTAG_36h11);
    static auto params     = cv::aruco::DetectorParameters::create();

    std::vector<int>                        all_ids;
    std::vector<std::vector<cv::Point2f>>   all_corners;
    cv::aruco::detectMarkers(gray, dictionary, all_corners, all_ids, params);

    std::vector<Detection> results;
    for (size_t i = 0; i < all_ids.size(); ++i) {
        if (std::find(TARGET_IDS.begin(), TARGET_IDS.end(), all_ids[i])
                == TARGET_IDS.end()) continue;

        cv::Point2f centre(0, 0);
        for (const auto& pt : all_corners[i]) centre += pt;
        centre *= 0.25f;

        results.push_back({all_ids[i], all_corners[i], centre});
    }
    return results;
}

// Draws detections onto frame and logs new IDs to stdout.
static void draw_detections(cv::Mat& frame, const std::vector<Detection>& detections)
{
    for (const auto& d : detections) {
        std::vector<std::vector<cv::Point2f>> c = {d.corners};
        std::vector<int>                      id = {d.id};
        cv::aruco::drawDetectedMarkers(frame, c, id);
        cv::circle(frame, d.centre, 4, cv::Scalar(0, 255, 0), -1);
        cv::putText(frame,
                    "ID " + std::to_string(d.id)
                    + " (" + std::to_string(int(d.centre.x))
                    + "," + std::to_string(int(d.centre.y)) + ")",
                    d.centre + cv::Point2f(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    std::string output_file;
    int         duration_sec = 0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-o" || arg == "--output") && i + 1 < argc)
            output_file = argv[++i];
        else if ((arg == "-d" || arg == "--duration") && i + 1 < argc)
            duration_sec = std::stoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") { print_usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << arg << "\n"; print_usage(argv[0]); return 1; }
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    PipelineCtx ctx;
    try {
        ctx = start_pipeline(output_file);
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n"; return 1;
    }

    std::string   csv_file = csv_path(output_file);
    std::ofstream csv      = open_csv(csv_file);
    if (!csv.is_open()) {
        std::cerr << "Failed to open CSV: " << csv_file << "\n"; return 1;
    }

    std::cout << "Detecting AprilTag 36h11 — target IDs: 92 93 94 95\n"
              << "Streams : Depth 848x480 | Color 848x480 | Accel | Gyro\n"
              << "Record  : " << (output_file.empty() ? "off" : output_file) << "\n"
              << "CSV     : " << csv_file << "\n"
              << (duration_sec > 0
                    ? "Duration: " + std::to_string(duration_sec) + "s\n"
                    : "Press q or Ctrl+C to stop.\n")
              << std::string(60, '-') << "\n";

    DepthFilters   filters;
    rs2::align     align_to_color(RS2_STREAM_COLOR);
    rs2::colorizer colorizer;

    auto       t_start     = std::chrono::steady_clock::now();
    long long  frame_count = 0;
    rs2_vector last_accel{}, last_gyro{};

    while (g_running) {
        if (duration_sec > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - t_start).count();
            if (elapsed >= duration_sec) break;
        }

        rs2::frameset frames;
        if (!ctx.pipe.poll_for_frames(&frames)) continue;

        if (auto a = frames.first_or_default(RS2_STREAM_ACCEL))
            last_accel = a.as<rs2::motion_frame>().get_motion_data();
        if (auto g = frames.first_or_default(RS2_STREAM_GYRO))
            last_gyro  = g.as<rs2::motion_frame>().get_motion_data();

        frames = align_to_color.process(frames);
        rs2::depth_frame raw_depth = frames.get_depth_frame();
        if (!raw_depth) continue;

        rs2::depth_frame filtered  = filters.process(raw_depth);
        cv::Mat          color_bgr = to_bgr(frames.get_color_frame());
        cv::Mat          depth_bgr = depth_to_mat(colorizer.colorize(filtered));

        // ── Marker detection ─────────────────────────────────────────────────
        cv::Mat gray;
        cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);
        auto detections = detect_markers(gray);
        draw_detections(color_bgr, detections);

        if (!detections.empty()) {
            double ts_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_start).count();
            log_detections(csv, frame_count, ts_ms, detections, filtered);
        }
        // ─────────────────────────────────────────────────────────────────────

        show_frames(color_bgr, depth_bgr);
        if (cv::waitKey(1) == 'q') break;

        // Depth stats
        double sum = 0.0; int valid = 0;
        const int w = filtered.get_width(), h = filtered.get_height();
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                float d = filtered.get_distance(x, y);
                if (d > 0.0f) { sum += d; ++valid; }
            }

        ++frame_count;
        if (frame_count % 5 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            print_stats(frame_count, elapsed, valid > 0 ? sum / valid : 0.0,
                        last_accel, last_gyro);
        }
    }

    ctx.pipe.stop();

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    std::cout << "\n" << std::string(60, '-') << "\n"
              << "Frames  : " << frame_count << "\n"
              << "Duration: " << std::fixed << std::setprecision(2) << elapsed << "s\n"
              << "CSV     : " << csv_file << "\n";
    if (!output_file.empty())
        std::cout << "Bag     : " << output_file << "\n";

    return 0;
}
