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
set(open_rgbd  
open_rgbd.cpp  

)  


# Create two executables  
add_executable(open_rgbd ${open_rgbd})  
# add_executable(open_rgb_2nd ${open_rgb_2nd})  

# Link libraries with both applications  
target_link_libraries(open_rgbd ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)  
# target_link_libraries(open_rgb_2nd ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)