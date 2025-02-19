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
set(SRCS_SLAM_6_DOF  
    slam_6_dof.cpp  
    raw2opencv.cpp  # Other files required for this executable  
)  

set(SRCS_SLAM_INFO  
    slam_info.cpp  
)
set(save_6_dof  
    save_6_dof.cpp  
)  

# Create two executables  
add_executable(slam_6_dof ${SRCS_SLAM_6_DOF})  
add_executable(slam_info ${SRCS_SLAM_INFO})  
add_executable(save_6_dof ${save_6_dof})  

# Link libraries with both applications  
target_link_libraries(slam_6_dof ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)  
target_link_libraries(slam_info ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(save_6_dof ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)