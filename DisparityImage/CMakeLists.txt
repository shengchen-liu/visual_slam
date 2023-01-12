cmake_minimum_required(VERSION 3.24)
project(DisparityImage)

set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS},
        "/usr/include/eigen3")

add_executable(Disparity disparity.cpp)
target_link_libraries(Disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable(Undistort undistort_image.cpp)
target_link_libraries(Undistort ${OpenCV_LIBS})

add_executable(GaussNewton gaussnewton.cpp)
target_link_libraries(GaussNewton ${OpenCV_LIBS})