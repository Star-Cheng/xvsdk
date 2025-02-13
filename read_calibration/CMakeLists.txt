cmake_minimum_required(VERSION 3.5)

project(read_calibration)

if ( WIN32 )
    set(xvsdk_DIR "../../cmake/xvsdk")
endif()
find_package( xvsdk REQUIRED )
set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )

set(SRCS read_calibration.cpp)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRCS} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${xvsdk_LIBRARIES} -pthread )

