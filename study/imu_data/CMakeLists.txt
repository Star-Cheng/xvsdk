cmake_minimum_required(VERSION 2.5)
project(slam_6_dof)

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
set(imu_get
imu_get.cpp
)

# Create two executables
add_executable(imu_get ${imu_get})

# Link libraries with both applications
target_link_libraries(imu_get ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
