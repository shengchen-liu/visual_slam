cmake_minimum_required(VERSION 3.10)
project(DrawTrajectory)

set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(DrawTrajectory draw_trajectory.cpp)
target_link_libraries(DrawTrajectory ${Pangolin_LIBRARIES})

find_package(Sophus REQUIRED)
target_link_libraries(DrawTrajectory Sophus::Sophus)





