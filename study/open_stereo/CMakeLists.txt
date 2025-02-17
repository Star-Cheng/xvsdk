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
set(SRCS_SLAM_6_DOF  
    open_stereo.cpp
)  
set(SRCS_SLAM_6_DOF  
    isheye_depth.cpp
)  

# Create two executables  
add_executable(open_stereo ${SRCS_SLAM_6_DOF})  
add_executable(fisheye_depth ${fisheye_depth})  


# Link libraries with both applications  
target_link_libraries(open_stereo ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)  
target_link_libraries(fisheye_depth ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)  