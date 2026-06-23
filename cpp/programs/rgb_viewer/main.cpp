#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 848, 480, RS2_FORMAT_BGR8, 30);

    try {
        pipe.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Streaming D456 RGB -- press 'q' to quit.\n";

    while (true)
    {
        rs2::frameset  frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        if (!color) continue;

        cv::Mat image(
            cv::Size(color.get_width(), color.get_height()),
            CV_8UC3,
            (void*)color.get_data(),
            cv::Mat::AUTO_STEP
        );

        cv::imshow("D456 RGB", image);
        if (cv::waitKey(1) == 'q') break;
    }

    pipe.stop();
    return 0;
}
