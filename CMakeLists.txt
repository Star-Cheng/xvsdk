cmake_minimum_required(VERSION 3.5)
project(xvsdk_samples)
set(CMAKE_CXX_STANDARD 11)

add_subdirectory( all_stream )
add_subdirectory( read_calibration )
add_subdirectory( object_detection )
add_subdirectory( cnn_on_rgb_and_slam )

