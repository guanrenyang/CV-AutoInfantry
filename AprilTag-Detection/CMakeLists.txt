cmake_minimum_required(VERSION 3.10)
project(AprilTag_Detection)

set(CMAKE_CXX_STANDARD 17)

# 如果有ccache则使用ccache加速编译
FIND_PROGRAM(CCACHE_FOUND ccache)
IF (CCACHE_FOUND)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    MESSAGE("-- Use ccache for compiler")
ENDIF ()

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system timer)

file(GLOB_RECURSE serial_src *.cc)
include_directories(include)

add_library(serial ${serial_src})

include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(StereoMV)
include_directories(StereoMV/include)

add_executable(run main.cpp)
target_link_libraries(run ${OpenCV_LIBS} StereoMV Boost::system Boost::timer serial)