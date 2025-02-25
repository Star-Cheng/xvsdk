cmake_minimum_required(VERSION 2.5)
project(open_rgbd)

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
set(open_tof
    open_tof.cpp
)
set(open_tof_ir
    open_tof_ir.cpp
)
set(save_tof_ply
    save_tof_ply.cpp
)

# Create two executables
add_executable(open_tof ${open_tof})
add_executable(open_tof_ir ${open_tof_ir})
add_executable(save_tof_ply ${save_tof_ply})

# Link libraries with both applications
target_link_libraries(open_tof ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(open_tof_ir ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(save_tof_ply ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)