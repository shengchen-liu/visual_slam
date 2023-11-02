#include "camera.h"
#include "essential_matrix.h"

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/eigen.hpp>


int main(int argc, char **argv){
    Camera camera = Camera(std::string(argv[1]));
    camera.GenerateFrames();

    Eigen::Matrix4d Twc1 = camera.GetFirstFrameTwc();
    Eigen::Matrix4d Twc2 = camera.GetSecondFrameTwc();

    std::vector<Eigen::Vector2d> points1 = camera.GetFirstFramePoints();
    std::vector<Eigen::Vector2d> points2 = camera.GetSecondFramePoints();

    std::vector<Eigen::Vector2d> normal_points1 = camera.GetFirstFrameNormalPoints();
    std::vector<Eigen::Vector2d> normal_points2 = camera.GetSecondFrameNormalPoints();

    Eigen::Matrix4d Tc1c2 = Twc1.inverse() * Twc2;
    Eigen::Matrix4d Tc2c1 = Twc2.inverse() * Twc1;

    Eigen::Matrix3d Rc2c1 = Tc2c1.block<3, 3>(0, 0);
    Eigen::Vector3d tc2c1 = Tc2c1.block<3, 1>(0, 3);

    double scale = tc2c1.norm();

    Eigen::Matrix3d E_gt = EssentialMatrixFromPose(Rc2c1, tc2c1);

    Eigen::Matrix3d E = EssentialMatrixEightPointEstimate(normal_points1, normal_points2);

    std::cout << "E gt "<<std::endl<<E_gt<<std::endl;
    std::cout << std::endl;
    std::cout << "E  "<<std::endl<<E<<std::endl;

    return 0;
}