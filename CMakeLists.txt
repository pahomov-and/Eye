cmake_minimum_required(VERSION 3.0)
project(EYE)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include(FindPkgConfig)

find_package(PkgConfig REQUIRED)

unset(CAMERA_INCLUDE_DIR CACHE)
find_path(
        CAMERA_INCLUDE_DIR
        NAMES "libcamera/libcamera.h"
        PATH_SUFFIXES libcamera
)
message(${CAMERA_INCLUDE_DIR})


find_library(CAMERA_LIBRARY
        NAMES camera
        PATH_SUFFIXES libcamera
        )
message(${CAMERA_LIBRARY})
message(${CAMERA_INCLUDE_DIR})

include_directories(
        .
        ${CAMERA_INCLUDE_DIR}
)

file(GLOB SRC
        *.cpp
        *.h
        )

add_executable(${PROJECT_NAME}
        ${SRC}
        Eye.cpp Eye.h EventLoop.cpp EventLoop.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${CAMERA_LIBRARY}
        tiff event pthread event_pthreads
        )
