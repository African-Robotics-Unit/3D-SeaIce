#include <librealsense2/rs.hpp>
#include <iostream>

int main() {
    rs2::pipeline pipeline;
    rs2::config config;

    config.enable_stream(RS2_STREAM_DEPTH,    0, 1280, 720, RS2_FORMAT_Z16, 15);
    config.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8,  15);
    config.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8,  15);

    try {
        rs2::pipeline_profile profile = pipeline.start(config);

        int prev_num = -1;
        for (int i = 0; i < 150; i++) {
            try {
                rs2::frameset frames = pipeline.wait_for_frames(5000);
                rs2::depth_frame depth = frames.get_depth_frame();

                if (!depth) {
                    std::cout << "NULL depth frame at iteration " << i << std::endl;
                    continue;
                }

                int n = static_cast<int>(depth.get_frame_number());

                if (prev_num != -1 && n != prev_num + 1) {
                    std::cout << "FRAME DROP: expected " << (prev_num + 1)
                              << ", got " << n << std::endl;
                }
                prev_num = n;

            } catch (const rs2::error& e) {
                std::cerr << "Frame capture error at iteration " << i
                          << ": " << e.what() << std::endl;
            }
        }

        pipeline.stop();

    } catch (const rs2::error& e) {
        std::cerr << "Pipeline error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}