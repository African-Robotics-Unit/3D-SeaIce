cmake_minimum_required(VERSION 3.15)
project(SensorSuite_RealSense VERSION 1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# C++14 is the minimum for librealsense2
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find installed librealsense2
find_package(realsense2 REQUIRED)

# Optional: find OpenCV if you want live display
# find_package(OpenCV REQUIRED)

# Build the recorder library
add_library(d456_recorder
    src/d456_recorder.cpp
)
target_include_directories(d456_recorder PUBLIC include/)
target_link_libraries(d456_recorder
    PUBLIC realsense2::realsense2
    # OpenCV::OpenCV   # uncomment if needed
)

# Build the main executable
add_executable(rs_record src/main.cpp)
target_link_libraries(rs_record PRIVATE d456_recorder)
