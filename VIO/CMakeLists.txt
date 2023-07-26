cmake_minimum_required(VERSION 3.10)
project(VIO)

set(CMAKE_CXX_STANDARD 14)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"        
)

# Sophus
find_package(Sophus REQUIRED)

# g2o
find_package(G2O REQUIRED)

# compute ORB features
add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

# GN-BA
add_executable(gn_ba GN-BA.cpp)
target_link_libraries(gn_ba 
                ${OpenCV_LIBS}
                Sophus::Sophus)

# 3d2d pose estimation pipeline
add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        Sophus::Sophus)


