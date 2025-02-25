cmake_minimum_required(VERSION 2.5)
project(open_stereo)

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
set(open_stereo
    open_stereo.cpp
)
set(open_stereo_keypoint
    open_stereo_keypoint.cpp
)
set(save_stereo
    save_stereo.cpp
)

# Create two executables
add_executable(open_stereo ${open_stereo})
add_executable(open_stereo_keypoint ${open_stereo_keypoint})
add_executable(save_stereo ${save_stereo})

# Link libraries with both applications
target_link_libraries(open_stereo ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(open_stereo_keypoint ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(save_stereo ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)