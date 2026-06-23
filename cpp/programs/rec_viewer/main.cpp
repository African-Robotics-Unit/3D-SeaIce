#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include <iomanip>

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running = false;
}

// Standard RealSense post-processing pipeline (all default settings)
struct PostProcessor {
    rs2::decimation_filter   decimation;
    rs2::disparity_transform to_disparity{true};
    rs2::spatial_filter      spatial;
    rs2::temporal_filter     temporal;
    rs2::disparity_transform to_depth{false};
    rs2::hole_filling_filter hole_fill;

    rs2::frame process(rs2::frame depth) {
        depth = decimation.process(depth);
        depth = to_disparity.process(depth);
        depth = spatial.process(depth);
        depth = temporal.process(depth);
        depth = to_depth.process(depth);
        depth = hole_fill.process(depth);
        return depth;
    }
};

int main(int argc, char* argv[]) {
    std::string output_file = "output.bag";
    int duration_sec = 0; // 0 = unlimited

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-o" || arg == "--output") && i + 1 < argc)
            output_file = argv[++i];
        else if ((arg == "-d" || arg == "--duration") && i + 1 < argc)
            duration_sec = std::stoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [-o output.bag] [-d duration_seconds]\n"
                      << "  -o  Output .bag file (default: output.bag)\n"
                      << "  -d  Recording duration in seconds (default: run until Ctrl+C)\n";
            return 0;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rs2::pipeline pipe;
    rs2::config cfg;

    // Default RealSense streams
    cfg.enable_stream(RS2_STREAM_DEPTH,  640, 480, RS2_FORMAT_Z16,  30);
    cfg.enable_stream(RS2_STREAM_COLOR,  640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);

    // Record raw sensor data to bag (post-processing applied on top in real time)
    cfg.enable_record_to_file(output_file);

    PostProcessor pp;
    rs2::colorizer colorizer;
    rs2::align align_to_color(RS2_STREAM_COLOR);

    std::cout << "Starting RealSense pipeline...\n";
    rs2::pipeline_profile profile;
    try {
        profile = pipe.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "Failed to start pipeline: " << e.what() << "\n";
        return 1;
    }

    // Apply High Density preset (must be set after pipeline starts)
    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
        depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
        std::cout << "Preset: High Density\n";
    } else {
        std::cout << "Preset: High Density not supported on this device, using defaults\n";
    }

    std::cout << "Recording raw + post-processed depth to: " << output_file << "\n";
    if (duration_sec > 0)
        std::cout << "Duration: " << duration_sec << "s\n";
    else
        std::cout << "Press Ctrl+C to stop.\n";
    std::cout << std::string(60, '-') << "\n";

    auto t_start = std::chrono::steady_clock::now();
    long long frame_count = 0;
    double min_depth_m = 1e9, max_depth_m = 0.0;
    rs2_vector last_accel{}, last_gyro{};

    while (g_running) {
        // Check duration
        if (duration_sec > 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - t_start).count();
            if (elapsed >= duration_sec) break;
        }

        rs2::frameset frames;
        if (!pipe.poll_for_frames(&frames))
            continue;

        if (auto accel = frames.first_or_default(RS2_STREAM_ACCEL))
            last_accel = accel.as<rs2::motion_frame>().get_motion_data();
        if (auto gyro = frames.first_or_default(RS2_STREAM_GYRO))
            last_gyro = gyro.as<rs2::motion_frame>().get_motion_data();

        // Align depth to color frame
        frames = align_to_color.process(frames);
        rs2::depth_frame raw_depth = frames.get_depth_frame();
        if (!raw_depth) continue;

        // Apply post-processing filters
        rs2::depth_frame filtered = pp.process(raw_depth).as<rs2::depth_frame>();

        // Gather per-frame statistics from filtered depth
        const int w = filtered.get_width();
        const int h = filtered.get_height();
        double frame_min = 1e9, frame_max = 0.0, sum = 0.0;
        int valid = 0;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float d = filtered.get_distance(x, y);
                if (d > 0.0f) {
                    if (d < frame_min) frame_min = d;
                    if (d > frame_max) frame_max = d;
                    sum += d;
                    ++valid;
                }
            }
        }
        if (frame_min < min_depth_m) min_depth_m = frame_min;
        if (frame_max > max_depth_m) max_depth_m = frame_max;

        // Colorize depth and show both windows
        rs2::frame colorized_depth = colorizer.colorize(filtered);
        cv::Mat depth_mat(cv::Size(colorized_depth.as<rs2::video_frame>().get_width(),
                                   colorized_depth.as<rs2::video_frame>().get_height()),
                          CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);

        rs2::video_frame color_frame = frames.get_color_frame();
        cv::Mat color_mat(cv::Size(color_frame.get_width(), color_frame.get_height()),
                          CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        // RGB → BGR for OpenCV display
        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);

        cv::imshow("Depth (colorized)", depth_mat);
        cv::imshow("RGB", color_mat);

        if (cv::waitKey(1) == 'q') break;

        ++frame_count;

        if (frame_count % 30 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            double fps = frame_count / elapsed;
            double avg = valid > 0 ? sum / valid : 0.0;
            std::cout << "\r"
                      << "frames: " << std::setw(6) << frame_count
                      << "  fps: " << std::fixed << std::setprecision(1) << std::setw(5) << fps
                      << "  depth avg: " << std::setprecision(3) << std::setw(6) << avg << "m"
                      << "  accel: [" << std::setprecision(2)
                      << last_accel.x << "," << last_accel.y << "," << last_accel.z << "]m/s²"
                      << "  gyro: [" << last_gyro.x << "," << last_gyro.y << "," << last_gyro.z << "]rad/s"
                      << "     " << std::flush;
        }
    }

    pipe.stop();

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();

    std::cout << "\n" << std::string(60, '-') << "\n"
              << "Recording stopped.\n"
              << "  Output file : " << output_file << "\n"
              << "  Total frames: " << frame_count << "\n"
              << "  Duration    : " << std::fixed << std::setprecision(2) << elapsed << "s\n"
              << "  Depth range : [" << std::setprecision(3)
              << min_depth_m << "m, " << max_depth_m << "m]\n"
              << "\nStreams recorded: Depth (Z16) | Color (RGB8) | Accel | Gyro\n"
              << "\nPost-processing filters applied (default settings):\n"
              << "  1. Decimation filter\n"
              << "  2. Depth -> Disparity transform\n"
              << "  3. Spatial filter\n"
              << "  4. Temporal filter\n"
              << "  5. Disparity -> Depth transform\n"
              << "  6. Hole-filling filter\n"
              << "\nNote: the .bag file stores raw sensor frames.\n"
              << "      Replay with this tool or rs-record/rs-convert to apply filters offline.\n";

    return 0;
}
