# MySLAM

# Class Design

## visual_odometry

Members:

| Type          | Name              | Comment |
| ------------- | ----------------- | ------- |
| bool          | inited_           |         |
| string        | config_file_path_ |         |
| Frontend::Ptr | frontend_         |         |
| Backend::Ptr  | backend_          |         |
| Map::Ptr      | map_              |         |
| Viewer::Ptr   | viewer_           |         |
| Dataset::Ptr  | dataset_          | dataset |

Methods:

| Method Name         | Return Value   | Comment                             |
| ------------------- | -------------- | ----------------------------------- |
| Init()              | bool           | do initialization things before run |
| Run()               | void           | start VO in the dataset             |
| Step()              | bool           | Make a step forward in dataset      |
| GetFrontendStatus() | FrontendStatus | Get FrontEnd status                 |
|                     |                |                                     |

## frontend

Members:

| Name                              | Type                    | Comment                                                      |
| --------------------------------- | ----------------------- | ------------------------------------------------------------ |
| status_                           | FrontendStatus          |                                                              |
| current_frame_                    | Frame::Ptr              |                                                              |
| last_frame_                       | Frame::Ptr              |                                                              |
| camera_right_                     | Camera::Ptr             |                                                              |
| camera_left_                      | Camera::Ptr             |                                                              |
| backend_                          | shared_ptr<Backend>     |                                                              |
| viewer_                           | shared_ptr<Viewer>      |                                                              |
| relative_motion_                  | SE3                     | relative motion between current and last frame.  Used to estimate current frame’s pose’s initial value |
| tracking_inliers_                 | int                     | number of inliers, used for testing new keyframes            |
| num_features_                     | int                     |                                                              |
| num_features_init_                | int                     |                                                              |
| num_features_tracking_            | int                     |                                                              |
| num_features_tracking_bad_        | int                     |                                                              |
| num_features_needed_for_keyframe_ | int                     |                                                              |
| gftt_                             | cv::Ptrcv::GFTTDetector | feature detector in opencv                                   |

| Method Name                                     | Return Value   | Comment                                                      |
| ----------------------------------------------- | -------------- | ------------------------------------------------------------ |
| AddFrame(Frame::Ptr frame)                      | bool           | 外部接口，添加一个帧并计算其定位结果                         |
| SetMap(Map::Ptr map)                            | void           | setters                                                      |
| SetBackend(std::shared_ptr<Backend> backend)    | void           |                                                              |
| SetViewer(std::shared_ptr<Viewer> viewer)       | void           |                                                              |
| SetCameras(Camera::Ptr left, Camera::Ptr right) | void           |                                                              |
| GetStatus()                                     | FrontendStatus |                                                              |
| Track()                                         | bool           | Track in normal mode                                         |
| Reset()                                         | bool           | Reset when lost                                              |
| TrackLastFrame()                                | int            | Track with last frame, return num of tracked points          |
| EstimateCurrentPose()                           | int            | estimate current frame's pose, return num of inliers         |
| InsertKeyframe()                                | bool           | set current frame as a keyframe and insert it into backend   |
| StereoInit()                                    | bool           | Try init the frontend with stereo images saved in current_frame_ |
| DetectFeatures()                                | int            | Detect features in left image in current_frame_, keypoints will be saved in current_frame_ |
| FindFeaturesInRight()                           | int            | Find the corresponding features in right image of current_frame_. |
| return num of features found                    |                |                                                              |
| BuildInitMap()                                  | bool           | Build the initial map with single image                      |
| TriangulateNewPoints()                          | int            | Triangulate the 2D points in current frame.                  |
| return num of triangulated points               |                |                                                              |
| etObservationsForKeyFrame()                     | void           | Set the features in keyframe as new observation of the map points |