cmake_minimum_required(VERSION 3.10)

project(essential)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen 3.0 REQUIRED)

find_package(Pangolin REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        camera
        estimator
)

add_executable(essential_matrix
        camera/camera.cpp
        estimator/essential_matrix.cpp
        main.cpp
 )

target_link_libraries(essential_matrix PRIVATE
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${Pangolin_LIBRARIES}
)
