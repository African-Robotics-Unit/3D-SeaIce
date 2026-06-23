#pragma once

#include <librealsense2/rs.hpp>
#include <string>
#include <atomic>
#include <thread>

class D456Recorder {
public:
    D456Recorder(const std::string& output_path, double duration_s,
                 bool high_density_preset = true);

    void start();
    void stop();
    bool is_running() const;

private:
    void acquisition_loop();
    void apply_preset();

    rs2::pipeline        pipe_;
    rs2::config          cfg_;
    std::string          output_path_;
    double               duration_s_;
    bool                 high_density_;
    std::atomic<bool>    running_{false};
    std::thread          thread_;
    std::exception_ptr   thread_exception_{nullptr};

    rs2::decimation_filter    dec_filter_;
    rs2::threshold_filter     thr_filter_;
    rs2::disparity_transform  depth_to_disparity_{true};
    rs2::disparity_transform  disparity_to_depth_{false};
    rs2::spatial_filter       spat_filter_;
    rs2::temporal_filter      temp_filter_;
    rs2::hole_filling_filter  hole_filter_;
};
