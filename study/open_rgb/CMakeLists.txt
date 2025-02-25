cmake_minimum_required(VERSION 2.5)
project(open_rgb)

# Find OpenCV
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DUSE_OPENCV)
else()
    message(FATAL_ERROR "OpenCV not found. ${PROJECT_NAME} will not be able to display images.")
endif()

# Find xvsdk
find_package(xvsdk REQUIRED)
include_directories(${xvsdk_INCLUDE_DIRS})

# Define source files for both executables
set(open_rgb
    open_rgb.cpp
)
set(open_rgb_thread
    open_rgb_thread.cpp
)
set(svae_rgb
    svae_rgb.cpp
)

# Create two executables
add_executable(open_rgb ${open_rgb})
add_executable(svae_rgb ${svae_rgb})
add_executable(open_rgb_thread ${open_rgb_thread})

# Link libraries with both applications
target_link_libraries(open_rgb ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(svae_rgb ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(open_rgb_thread ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)