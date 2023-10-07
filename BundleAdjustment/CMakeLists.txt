cmake_minimum_required(VERSION 3.5)

project(bundle_adjustment)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-O3 -std=c++14")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH /home/shengchen/visual_slam/3rdParty/g2o/cmake_modules ) 
# set(G2O_ROOT /usr/local/include/g2o) 

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(
    ${PROJECT_SOURCE_DIR} 
    ${EIGEN3_INCLUDE_DIR} 
    ${CSPARSE_INCLUDE_DIR}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    )

add_library(bal_common common.cpp)
add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)
add_executable(bundle_adjustment_ceres bundle_adjustment_ceres.cpp)

target_link_libraries(bundle_adjustment_ceres 
    ${CERES_LIBRARIES}
    bal_common)

target_link_libraries(bundle_adjustment_g2o 
    ${G2O_LIBS} 
    bal_common 
    Sophus::Sophus)
