cmake_minimum_required(VERSION 3.5)

project(multi-devices)

SET(SRC multi-devices.cpp)

if ( WIN32 )
    set(xvsdk_DIR "../../cmake/xvsdk")
endif()

find_package( xvsdk QUIET )
if( xvsdk_FOUND )
    message("xvsdk found .")
else()
    message("xvsdk is not found, so the local library will be linked.Or please install xvsdk correctly and reconfigure cmake.")
    set(xvsdk_DIR "${CMAKE_SOURCE_DIR}/../../../lib/cmake/xvsdk")
    set(xvsdk_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/../../../include")
    set(xvsdk_LIBRARIES "${CMAKE_SOURCE_DIR}/../../../lib")
    find_package( xvsdk REQUIRED )
endif()

set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )


ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} xvsdk )


