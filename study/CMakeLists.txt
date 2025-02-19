cmake_minimum_required(VERSION 2.5)  
project(study)  

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
set(ex1  
    ex1.cpp  
    raw2opencv.cpp  # Other files required for this executable  
)
set(ex2
    ex2.cpp  
    raw2opencv.cpp  # Other files required for this executable  
)
set(ex3
    ex3.cpp  
    raw2opencv.cpp  # Other files required for this executable  
)  

# Create two executables  
add_executable(ex1 ${ex1})  
add_executable(ex2 ${ex2})  
add_executable(ex3 ${ex3})  

# Link libraries with both applications  
target_link_libraries(ex1 ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(ex2 ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)
target_link_libraries(ex3 ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)  