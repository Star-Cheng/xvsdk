cmake_minimum_required(VERSION 3.5)

project(sgbm_demo)

if ( WIN32 )
    set(xvsdk_DIR "../../cmake/xvsdk")
endif()
find_package( xvsdk REQUIRED )
set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )

set(SRCS sgbm_demo.cc)

find_package(OpenCV QUIET)
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    add_definitions( -DUSE_OPENCV )
    set(SRCS ${SRCS} raw_to_opencv.cc )
    link_directories( ${OpenCV_LIB_PATH} )
else()
    message("OpenCV not found, ${PROJECT_NAME} will not be able to display images")
endif()

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRCS} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${xvsdk_LIBRARIES} ${OpenCV_LIBS} -pthread )

