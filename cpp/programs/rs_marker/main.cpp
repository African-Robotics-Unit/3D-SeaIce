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
#include <ctime>
#include <set>
#include <sys/stat.h>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static void print_usage(const char* prog)
{
    std::cout << "Usage: " << prog << " [-o output.bag] [-D output_dir] [-d duration_seconds] [-n frames] [-test] [-id]\n"
              << "  -o, --output    Record to .bag file (default: no recording)\n"
              << "                  In -test mode: video output path (default: detections_test.mp4)\n"
              << "  -D, --dir       Save bag + CSV into <output_dir>/<YYYYMMDD_HHMMSS>/\n"
              << "  -d, --duration  Duration in seconds     (default: run until q/Ctrl+C)\n"
              << "  -n, --frames    Stop after N frames      (default: no frame limit)\n"
              << "  -test           RGB-only: stream with marker overlays, save annotated video\n"
              << "  -id             RGB-only: detect ALL 36h11 tags, log unique IDs seen\n"
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

static PipelineCtx start_pipeline(const std::string& output_file, bool test_mode)
{
    PipelineCtx ctx;
    rs2::config cfg;
    if (test_mode) {
        cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, RS2_FORMAT_RGB8, 30);
    } else {
        cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16,        30);
        cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, RS2_FORMAT_RGB8,       30);
        cfg.enable_stream(RS2_STREAM_ACCEL,             RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO,              RS2_FORMAT_MOTION_XYZ32F);
        if (!output_file.empty())
            cfg.enable_record_to_file(output_file);
    }
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

// Derives ply path from bag name: "capture.bag" → "capture.ply".
// Falls back to "best_frame.ply" when not recording.
static std::string ply_path(const std::string& bag)
{
    if (bag.empty()) return "best_frame.ply";
    auto dot = bag.rfind('.');
    return (dot == std::string::npos ? bag : bag.substr(0, dot)) + ".ply";
}

// Directory portion of a path, or "." if there is none.
static std::string dir_of(const std::string& path)
{
    auto slash = path.rfind('/');
    return slash == std::string::npos ? "." : path.substr(0, slash);
}

// Writes the IMU sample captured alongside the selected best frame.
static void write_imu_csv(const std::string& path, long long frame, double timestamp_ms,
                          rs2_vector accel, rs2_vector gyro)
{
    std::ofstream f(path);
    f << "frame,timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
    f << std::fixed << std::setprecision(6)
      << frame << "," << timestamp_ms << ","
      << accel.x << "," << accel.y << "," << accel.z << ","
      << gyro.x  << "," << gyro.y  << "," << gyro.z  << "\n";
}

// depth_arg may be an empty frame (test mode) — depth_m written as 0.0 in that case.
static void log_detections(std::ofstream& f, long long frame, double timestamp_ms,
                            const std::vector<Detection>& detections,
                            const rs2::frame& depth_arg)
{
    rs2::depth_frame depth = depth_arg.as<rs2::depth_frame>();
    for (const auto& d : detections) {
        float depth_m = 0.0f;
        if (depth) {
            depth_m = depth.get_distance(
                std::max(0, std::min((int)d.centre.x, depth.get_width()  - 1)),
                std::max(0, std::min((int)d.centre.y, depth.get_height() - 1)));
        }
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

static const std::vector<int> TARGET_IDS = {0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 93, 94, 95};

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

// Detects every 36h11 tag visible — no ID filter. Used by -id mode.
static std::vector<Detection> detect_all_markers(const cv::Mat& gray)
{
    static auto dictionary = cv::aruco::getPredefinedDictionary(
                                 cv::aruco::DICT_APRILTAG_36h11);
    static auto params     = cv::aruco::DetectorParameters::create();

    std::vector<int>                      all_ids;
    std::vector<std::vector<cv::Point2f>> all_corners;
    cv::aruco::detectMarkers(gray, dictionary, all_corners, all_ids, params);

    std::vector<Detection> results;
    for (size_t i = 0; i < all_ids.size(); ++i) {
        cv::Point2f centre(0, 0);
        for (const auto& pt : all_corners[i]) centre += pt;
        centre *= 0.25f;
        results.push_back({all_ids[i], all_corners[i], centre});
    }
    return results;
}

// ═════════════════════════════════════════════════════════════════════════════

// ── Session directory helpers ─────────────────────────────────────────────────

static std::string timestamp_str()
{
    std::time_t t  = std::time(nullptr);
    std::tm     tm = *std::localtime(&t);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

// Creates <parent>/<YYYYMMDD_HHMMSS>/ and returns the session directory path.
// Creates <parent> first if it doesn't exist.
static std::string make_session_dir(const std::string& parent)
{
    mkdir(parent.c_str(), 0755);  // no-op if already exists
    std::string dir = parent + "/" + timestamp_str();
    if (mkdir(dir.c_str(), 0755) != 0) {
        std::cerr << "Failed to create session directory: " << dir << "\n";
        std::exit(1);
    }
    return dir;
}

// ─────────────────────────────────────────────────────────────────────────────

// ── Test mode helpers ─────────────────────────────────────────────────────────

// Derive video output path from the -o argument (if given), else use default.
static std::string video_path(const std::string& arg)
{
    if (!arg.empty()) return arg;
    return "detections_test.mp4";
}

static cv::VideoWriter open_video(const std::string& path, int w, int h, double fps)
{
    cv::VideoWriter writer(path,
                           cv::VideoWriter::fourcc('m','p','4','v'),
                           fps, cv::Size(w, h));
    return writer;
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    std::string output_file;
    std::string output_dir;
    int         duration_sec  = 0;
    long long   frame_limit   = 0;   // 0 = no limit
    bool        test_mode     = false;
    bool        id_mode       = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-o" || arg == "--output") && i + 1 < argc)
            output_file = argv[++i];
        else if ((arg == "-D" || arg == "--dir") && i + 1 < argc)
            output_dir = argv[++i];
        else if ((arg == "-d" || arg == "--duration") && i + 1 < argc)
            duration_sec = std::stoi(argv[++i]);
        else if ((arg == "-n" || arg == "--frames") && i + 1 < argc)
            frame_limit = std::stoll(argv[++i]);
        else if (arg == "-test" || arg == "--test")
            test_mode = true;
        else if (arg == "-id" || arg == "--id")
            id_mode = true;
        else if (arg == "-h" || arg == "--help") { print_usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << arg << "\n"; print_usage(argv[0]); return 1; }
    }

    if (id_mode && test_mode) {
        std::cerr << "Error: -id and -test cannot be used together.\n"; return 1;
    }
    if (!output_dir.empty() && !output_file.empty()) {
        std::cerr << "Error: -o and -D cannot be used together.\n"; return 1;
    }
    if (!output_dir.empty()) {
        std::string session_dir = make_session_dir(output_dir);
        std::string ts_name     = session_dir.substr(session_dir.rfind('/') + 1);
        output_file = session_dir + "/" + ts_name + ".bag";
        std::cout << "Session : " << session_dir << "\n";
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    PipelineCtx ctx;
    try {
        ctx = start_pipeline((test_mode || id_mode) ? "" : output_file,
                             test_mode || id_mode);
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n"; return 1;
    }

    std::string   csv_file = id_mode ? "" : csv_path(test_mode ? "" : output_file);
    std::ofstream csv;
    if (!id_mode) {
        csv = open_csv(csv_file);
        if (!csv.is_open()) {
            std::cerr << "Failed to open CSV: " << csv_file << "\n"; return 1;
        }
    }

    // ── Best-frame outputs: only meaningful in normal mode (depth + IMU) ──────
    bool        capture_best = !test_mode && !id_mode;
    std::string ply_file     = capture_best ? ply_path(output_file) : "";
    std::string imu_file     = capture_best
        ? (output_file.empty() ? "imu.csv" : dir_of(output_file) + "/imu.csv")
        : "";

    // ── Test mode: open video writer ──────────────────────────────────────────
    cv::VideoWriter video_writer;
    std::string     vid_file;
    if (test_mode) {
        vid_file     = video_path(output_file);
        video_writer = open_video(vid_file, 848, 480, 5.0);
        if (!video_writer.isOpened()) {
            std::cerr << "Failed to open video writer: " << vid_file << "\n"; return 1;
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

    if (id_mode) {
        std::cout << "ID MODE — RGB-only, detect ALL AprilTag 36h11 tags\n"
                  << "Wave each tag in front of the camera to confirm it works.\n"
                  << "New unique IDs will be printed as they are detected.\n"
                  << "Stream  : Color 1280x720\n"
                  << (duration_sec > 0
                        ? "Duration: " + std::to_string(duration_sec) + "s\n"
                        : "Press q or Ctrl+C to stop.\n")
                  << std::string(60, '-') << "\n";
    } else if (test_mode) {
        std::cout << "TEST MODE — RGB-only, no depth/IMU\n"
                  << "Detecting AprilTag 36h11 — ALL tags\n"
                  << "Stream  : Color 848x480\n"
                  << "Video   : " << vid_file  << "\n"
                  << "CSV     : " << csv_file  << "\n"
                  << (duration_sec > 0
                        ? "Duration: " + std::to_string(duration_sec) + "s\n"
                        : "Press q or Ctrl+C to stop.\n")
                  << std::string(60, '-') << "\n";
    } else {
        std::cout << "Detecting AprilTag 36h11 — ALL tags\n"
                  << "Streams : Depth 848x480 | Color 848x480 | Accel | Gyro\n"
                  << "Record  : " << (output_file.empty() ? "off" : output_file) << "\n"
                  << "CSV     : " << csv_file << "\n"
                  << "PLY     : " << ply_file << "  (best frame — least holes)\n"
                  << "IMU     : " << imu_file << "\n"
                  << (duration_sec > 0
                        ? "Duration: " + std::to_string(duration_sec) + "s\n"
                        : "")
                  << (frame_limit > 0
                        ? "Frames  : " + std::to_string(frame_limit) + "\n"
                        : "")
                  << (duration_sec == 0 && frame_limit == 0
                        ? "Press q or Ctrl+C to stop.\n"
                        : "")
                  << std::string(60, '-') << "\n";
    }

    DepthFilters   filters;
    rs2::align     align_to_color(RS2_STREAM_COLOR);
    rs2::colorizer colorizer;
    rs2::pointcloud pc;

    auto       t_start     = std::chrono::steady_clock::now();
    long long  frame_count = 0;
    rs2_vector last_accel{}, last_gyro{};
    std::set<int> seen_ids;  // used by -id mode

    // ── Best-frame tracking (normal mode): most valid depth pixels = fewest holes ──
    rs2::depth_frame best_depth{rs2::frame{}};
    rs2::video_frame best_color{rs2::frame{}};
    rs2_vector       best_accel{}, best_gyro{};
    long long        best_frame_idx    = -1;
    double           best_ts_ms        = 0.0;
    int              best_valid_pixels = -1;

    while (g_running) {
        if (duration_sec > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - t_start).count();
            if (elapsed >= duration_sec) break;
        }
        if (frame_limit > 0 && frame_count >= frame_limit) break;

        rs2::frameset frames;
        if (!ctx.pipe.poll_for_frames(&frames)) continue;

        if (id_mode) {
            // ── ID mode loop: detect all 36h11 tags, log unique IDs ───────────
            rs2::video_frame color_frame = frames.get_color_frame();
            if (!color_frame) continue;

            cv::Mat color_bgr = to_bgr(color_frame);
            cv::Mat gray;
            cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);
            auto detections = detect_all_markers(gray);

            for (const auto& d : detections) {
                if (seen_ids.insert(d.id).second) {
                    std::cout << "[NEW] ID: " << d.id
                              << "  (total unique: " << seen_ids.size() << ")\n"
                              << std::flush;
                }
            }

            draw_detections(color_bgr, detections);

            // Overlay unique-ID list in top-left corner
            cv::putText(color_bgr,
                        "Unique IDs seen: " + std::to_string(seen_ids.size()),
                        cv::Point(10, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 220, 255), 2);
            int y = 58;
            for (int id : seen_ids) {
                cv::putText(color_bgr, "  " + std::to_string(id),
                            cv::Point(10, y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 255), 1);
                y += 22;
            }

            cv::imshow("AprilTag ID scanner (36h11)", color_bgr);
            if (cv::waitKey(1) == 'q') break;
            // ─────────────────────────────────────────────────────────────────
        } else if (test_mode) {
            // ── Test mode loop: RGB only ──────────────────────────────────────
            rs2::video_frame color_frame = frames.get_color_frame();
            if (!color_frame) continue;

            cv::Mat color_bgr = to_bgr(color_frame);

            cv::Mat gray;
            cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);
            auto detections = detect_markers(gray);
            draw_detections(color_bgr, detections);

            video_writer.write(color_bgr);

            if (!detections.empty()) {
                double ts_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_start).count();
                log_detections(csv, frame_count, ts_ms, detections, rs2::frame{});
            }

            cv::imshow("RGB + markers (test)", color_bgr);
            if (cv::waitKey(1) == 'q') break;
            // ─────────────────────────────────────────────────────────────────
        } else {
            // ── Normal mode loop ──────────────────────────────────────────────
            if (auto a = frames.first_or_default(RS2_STREAM_ACCEL))
                last_accel = a.as<rs2::motion_frame>().get_motion_data();
            if (auto g = frames.first_or_default(RS2_STREAM_GYRO))
                last_gyro  = g.as<rs2::motion_frame>().get_motion_data();

            frames = align_to_color.process(frames);
            rs2::depth_frame raw_depth = frames.get_depth_frame();
            if (!raw_depth) continue;

            rs2::depth_frame filtered   = filters.process(raw_depth);
            rs2::video_frame color_frame = frames.get_color_frame();
            cv::Mat          color_bgr  = to_bgr(color_frame);
            cv::Mat          depth_bgr  = depth_to_mat(colorizer.colorize(filtered));

            double ts_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_start).count();

            // ── Marker detection ─────────────────────────────────────────────
            cv::Mat gray;
            cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);
            auto detections = detect_all_markers(gray);
            draw_detections(color_bgr, detections);

            if (!detections.empty())
                log_detections(csv, frame_count, ts_ms, detections, filtered);
            // ─────────────────────────────────────────────────────────────────

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

            // Best-frame selection: most valid depth pixels == fewest holes
            if (valid > best_valid_pixels) {
                best_valid_pixels = valid;
                best_depth        = filtered;
                best_color        = color_frame;
                best_accel        = last_accel;
                best_gyro         = last_gyro;
                best_frame_idx    = frame_count;
                best_ts_ms        = ts_ms;
            }

            if (frame_count % 5 == 0) {
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_start).count();
                print_stats(frame_count, elapsed, valid > 0 ? sum / valid : 0.0,
                            last_accel, last_gyro);
            }
            // ─────────────────────────────────────────────────────────────────
        }

        ++frame_count;
    }

    ctx.pipe.stop();
    if (test_mode) video_writer.release();

    // ── Export best frame (most valid depth pixels = fewest holes) ────────────
    if (capture_best && best_valid_pixels >= 0) {
        pc.map_to(best_color);
        rs2::points points = pc.calculate(best_depth);
        points.export_to_ply(ply_file, best_color);
        write_imu_csv(imu_file, best_frame_idx, best_ts_ms, best_accel, best_gyro);
    }
    // ─────────────────────────────────────────────────────────────────────────

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    std::cout << "\n" << std::string(60, '-') << "\n"
              << "Frames  : " << frame_count << "\n"
              << "Duration: " << std::fixed << std::setprecision(2) << elapsed << "s\n";

    if (id_mode) {
        std::cout << "Unique IDs detected (" << seen_ids.size() << "): ";
        for (int id : seen_ids) std::cout << id << " ";
        std::cout << "\n";
    } else {
        std::cout << "CSV     : " << csv_file << "\n";
        if (test_mode) {
            std::cout << "Video   : " << vid_file << "\n";
        } else {
            if (!output_file.empty())
                std::cout << "Bag     : " << output_file << "\n";
            if (best_valid_pixels >= 0) {
                std::cout << "PLY     : " << ply_file
                          << "  (frame " << best_frame_idx << ")\n"
                          << "IMU     : " << imu_file << "\n";
            } else {
                std::cout << "PLY     : (skipped — no depth frame captured)\n";
            }
        }
    }

    return 0;
}
