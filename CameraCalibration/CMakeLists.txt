cmake_minimum_required(VERSION 3.20)

project(camera_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
    ./include
)

set (
    TARGET_SRC
    ./src/main.cpp
    ./src/SingleCamera.cpp)

add_executable (
    camera_calibration
    ${TARGET_SRC}
)

target_link_libraries (camera_calibration Eigen3::Eigen)