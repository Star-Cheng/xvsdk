cmake_minimum_required(VERSION 2.5)  
project(study)  
# set(CMAKE_BUILD_TYPE Debug)
# Find OpenCV  
find_package(OpenCV QUIET)  
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if(OpenCV_FOUND)  
    include_directories(${OpenCV_INCLUDE_DIRS})  
    add_definitions(-DUSE_OPENCV)  
else()  
    message(FATAL_ERROR "OpenCV not found. ${PROJECT_NAME} will not be able to display images.")  
endif()  

# Find xvsdk  
find_package(xvsdk REQUIRED)  
include_directories(${xvsdk_INCLUDE_DIRS})  

# Define source files for executables  
set(ex1_sources  
    ex1.cpp  
)
set(ex2_sources
    ex2.cpp  
)
set(ex3_sources
    ex3.cpp  
)

# Create a static library for raw2opencv
add_library(raw2opencv STATIC raw2opencv.cpp)

# Create executables  
add_executable(ex1 ${ex1_sources})  
add_executable(ex2 ${ex2_sources})  
add_executable(ex3 ${ex3_sources})  

# Link the raw2opencv library with each executable
target_link_libraries(ex1 raw2opencv ${OpenCV_LIBS} ${xvsdk_LIBRARIES} ${PCL_LIBRARIES} pthread)
target_link_libraries(ex2 raw2opencv ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(ex3 raw2opencv ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)