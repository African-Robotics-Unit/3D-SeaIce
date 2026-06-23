#include "d456_recorder.hpp"
#include <iostream>
#include <chrono>

D456Recorder::D456Recorder(const std::string& output_path,
                           double duration_s,
                           bool high_density_preset)
    : output_path_(output_path)
    , duration_s_(duration_s)
    , high_density_(high_density_preset)
{
    cfg_.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16,  30);
    cfg_.enable_stream(RS2_STREAM_COLOR, 0, 848, 480, RS2_FORMAT_RGB8, 30);
    cfg_.enable_record_to_file(output_path_);
}

void D456Recorder::start()
{
    running_ = true;
    thread_  = std::thread(&D456Recorder::acquisition_loop, this);
}

void D456Recorder::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
    try { pipe_.stop(); } catch (...) {}
    if (thread_exception_)
        std::rethrow_exception(thread_exception_);
}

bool D456Recorder::is_running() const { return running_.load(); }

void D456Recorder::acquisition_loop()
{
    try {
        auto profile = pipe_.start(cfg_);
        apply_preset();

        auto t0 = std::chrono::steady_clock::now();
        while (running_)
        {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed >= duration_s_) break;

            rs2::frameset frames = pipe_.wait_for_frames();
            rs2::frame depth     = frames.get_depth_frame();
            if (!depth) continue;

            depth = dec_filter_.process(depth);
            depth = thr_filter_.process(depth);
            depth = depth_to_disparity_.process(depth);
            depth = spat_filter_.process(depth);
            depth = temp_filter_.process(depth);
            depth = disparity_to_depth_.process(depth);
            depth = hole_filter_.process(depth);
        }
    } catch (...) {
        thread_exception_ = std::current_exception();
    }
    running_ = false;
}

void D456Recorder::apply_preset()
{
    auto sensor = pipe_.get_active_profile().get_device().query_sensors()[0];
    if (high_density_ && sensor.supports(RS2_OPTION_VISUAL_PRESET))
        sensor.set_option(RS2_OPTION_VISUAL_PRESET,
                          static_cast<float>(RS2_RS400_VISUAL_PRESET_HIGH_DENSITY));
}
