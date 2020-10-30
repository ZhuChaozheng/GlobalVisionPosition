cmake_minimum_required(VERSION 2.8)
project(OpencvProcess)


set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -pthread")


# set directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)


#set(OpenCV_DIR "/home/nvidia/Downloads/opencv-3.1.0/build")
include_directories("/opt/MVS/include")
link_directories("/opt/MVS/lib/aarch64")

find_package(OpenCV REQUIRED)
message(STATUS "version: ${OpenCV_VERSION}")

include_directories( ${OpenCV_INCLUDE_DIRS} )


set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
  	MvCameraControl
)

################## dependencies ################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)

