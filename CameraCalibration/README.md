# Camera Calibration

## Description

The C++ version of the SingleCamera class implements the main steps of camera calibration. In camera calibration, we need to find the *internal* and *external* matrices of the camera, as well as the *transformation* matrix between the camera and the world coordinate system. The main function of this class is to calculate these matrices.

The SingleCamera class has three variables that need to be input: world_coor, pixel_coor and point_num, which store world coordinates, pixel coordinates and point numbers respectively. The class contains several member functions, including:

- composeP(): Generate matrix P and convert it into a suitable form so that Pm=0.
- svdP(): Perform singular value decomposition on matrix P to obtain M=[A b].
- workInAndOut(): Calculate the camera internal parameter matrix K and external parameter matrix [R t] through M.
- selfcheck(): Used to verify whether the generated internal and external parameters are correct.

The SingleCamera class can calibrate the camera by calling the above functions to calculate the internal and external parameters of the camera. You can use the selfcheck() function to verify that the calculated transformation matrix is correct.

This example selects a stereo calibration object picture on the Internet

![demo](demo.jpg)

## Output 

```
pixels given = 
369 297
531 484
640 468
646 333
556 194

pixels projected = 
369.955 296.987
530.973 482.574
639.758 468.575
647.492 332.448
559.84 194.248
error = 0.936957
```

## How to run

Compile：  
cd CameraCalibration  
mkdir build  
cd build  
cmake ..  
make   

运行：

./camera_calib
