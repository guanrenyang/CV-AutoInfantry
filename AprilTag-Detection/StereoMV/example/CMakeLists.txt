add_compile_definitions(EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Boost REQUIRED COMPONENTS system timer)

add_executable(example example.cpp)
target_link_libraries(example StereoMV Boost::system Boost::timer)

add_executable(calib calib.cpp)
target_link_libraries(calib StereoMV)