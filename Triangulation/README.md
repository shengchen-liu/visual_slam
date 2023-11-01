# Triangulation

![](./result.gif)

This project provides you with a camera simulation environment. By loading a small house model and giving the camera parameters and motion trajectories, we can obtain the projected image of the small house in the camera and the corresponding camera's posture. This project requires  to calculate the position of a two-dimensional point in three dimensions when the pose of the camera and the points on the image are known, and display it in pangolin visualization.

What everyone needs to implement are the two functions TriangulatePoints and TriangulatePoint in estimator/triangulation.cpp. 

1. first implement the TriangulatePoint function to realize the estimation function of a single three-dimensional point.

2. implement the TriangulatePoints function to traverse all points to realize the estimation function of the three-dimensional points of all points.

- Third-party libraries：

1. Eigen3
2. OpenCV 4.x
3. Pangolin 0.6

- Compile
mkdir build 
cd build
cmake ..
make -j4

- run：
./triangulate ../data/house.txt