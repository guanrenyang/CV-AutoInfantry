cmake_minimum_required(VERSION 3.5)
project(StereoMV)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)

find_package(fmt REQUIRED)

include_directories(include)

aux_source_directory(src StereoMV_src)
add_library(StereoMV SHARED ${StereoMV_src})
target_link_libraries(StereoMV MVSDK ${OpenCV_LIBS} fmt::fmt)

option(BUILD_EXAMPLE "build the example" OFF)

if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif ()