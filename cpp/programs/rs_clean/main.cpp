#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <atomic>
#include <chrono>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static void print_usage(const char* prog)
{
    std::cout << "Usage: " << prog << " [-o output.bag] [-d duration_seconds]\n"
              << "  -o, --output    Output .bag file       (default: record.bag)\n"
              << "  -d, --duration  Duration in seconds    (default: run until Ctrl+C)\n"
              << "  -h, --help      Show this message\n";
}

// Post-processing pipeline applied to depth frames before display/analysis.
// Matches the filter chain used by rec_viewer (no threshold filter — it was
// silently zeroing frames and producing invalid bags in rs_record).
struct DepthFilters {
    rs2::decimation_filter   decimation;
    rs2::disparity_transform to_disparity{true};
    rs2::spatial_filter      spatial;
    rs2::temporal_filter     temporal;
    rs2::disparity_transform to_depth{false};
    rs2::hole_filling_filter hole_fill;

    rs2::frame process(rs2::frame depth)
    {
        depth = decimation.process(depth);
        depth = to_disparity.process(depth);
        depth = spatial.process(depth);
        depth = temporal.process(depth);
        depth = to_depth.process(depth);
        depth = hole_fill.process(depth);
        return depth;
    }
};

int main(int argc, char* argv[])
{
    std::string output_file  = "record.bag";
    int         duration_sec = 0;           // 0 = run until Ctrl+C

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

    rs2::pipeline pipe;
    rs2::config   cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16,        5);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 848, 480, RS2_FORMAT_RGB8,       5);
    cfg.enable_stream(RS2_STREAM_ACCEL,             RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO,              RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_record_to_file(output_file);

    rs2::pipeline_profile profile;
    try {
        profile = pipe.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n";
        return 1;
    }

    // Apply High Density preset after pipeline starts
    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
        depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET,
                                RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);

    std::cout << "Recording to : " << output_file << "\n"
              << "Streams      : Depth 848x480 | Color 848x480 | Accel | Gyro\n"
              << "Preset       : High Density\n";
    if (duration_sec > 0)
        std::cout << "Duration     : " << duration_sec << "s\n";
    else
        std::cout << "Duration     : until Ctrl+C\n";
    std::cout << std::string(60, '-') << "\n";

    DepthFilters   filters;
    rs2::align     align_to_color(RS2_STREAM_COLOR);
    rs2::colorizer colorizer;

    auto          t_start     = std::chrono::steady_clock::now();
    long long     frame_count = 0;
    rs2_vector    last_accel{}, last_gyro{};

    while (g_running) {
        if (duration_sec > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - t_start).count();
            if (elapsed >= duration_sec) break;
        }

        rs2::frameset frames;
        if (!pipe.poll_for_frames(&frames)) continue;

        if (auto a = frames.first_or_default(RS2_STREAM_ACCEL))
            last_accel = a.as<rs2::motion_frame>().get_motion_data();
        if (auto g = frames.first_or_default(RS2_STREAM_GYRO))
            last_gyro  = g.as<rs2::motion_frame>().get_motion_data();

        frames = align_to_color.process(frames);
        rs2::depth_frame raw_depth = frames.get_depth_frame();
        if (!raw_depth) continue;

        rs2::depth_frame filtered = filters.process(raw_depth).as<rs2::depth_frame>();

        // Display
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::frame       depth_color = colorizer.colorize(filtered);

        cv::Mat color_mat(color_frame.get_height(), color_frame.get_width(),
                          CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_mat(depth_color.as<rs2::video_frame>().get_height(),
                          depth_color.as<rs2::video_frame>().get_width(),
                          CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);

        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
        cv::imshow("RGB",              color_mat);
        cv::imshow("Depth (filtered)", depth_mat);
        if (cv::waitKey(1) == 'q') break;

        // Gather depth statistics from filtered frame
        const int w = filtered.get_width(), h = filtered.get_height();
        double sum = 0.0; int valid = 0;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                float d = filtered.get_distance(x, y);
                if (d > 0.0f) { sum += d; ++valid; }
            }

        ++frame_count;

        if (frame_count % 5 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            double avg = valid > 0 ? sum / valid : 0.0;
            std::cout << "\r"
                      << "frames: "   << std::setw(6) << frame_count
                      << "  fps: "    << std::fixed << std::setprecision(1)
                                      << std::setw(5) << frame_count / elapsed
                      << "  depth: "  << std::setprecision(3) << std::setw(6) << avg << "m"
                      << "  accel: [" << std::setprecision(2)
                                      << last_accel.x << ","
                                      << last_accel.y << ","
                                      << last_accel.z << "]"
                      << "  gyro: ["  << last_gyro.x  << ","
                                      << last_gyro.y  << ","
                                      << last_gyro.z  << "]"
                      << "     " << std::flush;
        }
    }

    pipe.stop();

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();

    std::cout << "\n" << std::string(60, '-') << "\n"
              << "Output  : " << output_file  << "\n"
              << "Frames  : " << frame_count   << "\n"
              << "Duration: " << std::fixed << std::setprecision(2) << elapsed << "s\n";

    return 0;
}
