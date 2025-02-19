cmake_minimum_required(VERSION 2.5)
project(compose_stereo)
set(CMAKE_BUILD_TYPE Debug)
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

# Define source files for both executables
set(compose_stereo
    compose_stereo.cpp
)

# set(save_stereo
# save_stereo.cpp
# )

# Create two executables
add_executable(compose_stereo ${compose_stereo})

# add_executable(save_stereo ${save_stereo})

# Link libraries with both applications
target_link_libraries(compose_stereo ${OpenCV_LIBS} ${xvsdk_LIBRARIES} ${PCL_LIBRARIES} pthread)
# target_link_libraries (compose_stereo ${PCL_LIBRARIES})
# install(TARGETS pcl_test RUNTIME DESTINATION bin)

# install(TARGETS compose_stereo RUNTIME DESTINATION bin)
# target_link_libraries(save_stereo ${OpenCV_LIBS} ${xvsdk_LIBRARIES} pthread)