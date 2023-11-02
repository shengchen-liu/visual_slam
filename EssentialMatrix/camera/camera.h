#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

// #include <opencv2/opencv2.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "types.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;

using Line = std::pair<int, int>;
using Lines = std::vector<Line>;

struct MotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;
};


class Frame {
public:
    std::vector<Eigen::Vector2d> points;
    std::vector<Eigen::Vector2d> normal_points;

    MotionData pose;

    int frame_idx;

    cv::Mat image_;
};

class Camera {
public:
    Camera(std::string model_path_);

    // dataset 
    std::string model_path;

    // time
    int cam_frequency = 3;
    double cam_timestep = 1./cam_frequency;
    double t_start = 0.;
    double t_end = 20;  //  20 s

    // noise
    double pixel_noise = 1;              // 1 pixel noise

    // 相机内参
    double fx = 460;
    double fy = 460;
    double cx = 320;
    double cy = 240;
    double image_w = 640;
    double image_h = 480;

    Eigen::Matrix3d intrinsic_matrix;

    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body
    
    std::vector<MotionData> motion_datas;
    std::vector<Frame> frames;

    MotionData MotionModel(double t);

    // euler2Rotation:  body frame to interitail frame
    Eigen::Matrix3d euler2Rotation(Eigen::Vector3d  eulerAngles);

    void GenerateFrames();

    Points ModelPoints;
    Lines ModelLines;

    std::vector<Eigen::Matrix4d> Twbs;
    std::vector<Eigen::Matrix4d> Twcs;

    int first_frame_has_whole_points = -1;
    int second_frame_has_whole_points = -1;

    Eigen::Matrix4d GetFirstFrameTwc();
    Eigen::Matrix4d GetSecondFrameTwc();

    std::vector<Eigen::Vector2d> GetFirstFramePoints();
    std::vector<Eigen::Vector2d> GetSecondFramePoints();

    std::vector<Eigen::Vector2d> GetFirstFrameNormalPoints();
    std::vector<Eigen::Vector2d> GetSecondFrameNormalPoints();
};