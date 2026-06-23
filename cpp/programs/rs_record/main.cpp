#include "d456_recorder.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char* argv[])
{
    std::string outfile = (argc > 1) ? argv[1] : "d455_record.bag";
    double duration     = (argc > 2) ? std::stod(argv[2]) : 30.0;

    try {
        D456Recorder recorder(outfile, duration, /*high_density=*/true);
        recorder.start();
        std::cout << "Recording " << duration << "s to " << outfile << " ...\n";
        while (recorder.is_running())
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        recorder.stop();
        std::cout << "Done.\n";
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
