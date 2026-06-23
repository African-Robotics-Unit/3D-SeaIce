cmake_minimum_required(VERSION 3.15)
project(rgb_test VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reuse the FindRealsense2 module from the parent cpp/ folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rgb_test main.cpp)
target_link_libraries(rgb_test
    PRIVATE realsense2::realsense2
    PRIVATE ${OpenCV_LIBS}
)
