#pragma once

#include <iostream>
#include <Eigen/Dense>

class SingleCamera {

public:
    SingleCamera(Eigen::MatrixXf worldCoord, Eigen::MatrixXf pixelCoord, int n, const float signRho, const float signR3, const bool verbose)
            : worldCoord(worldCoord),
              pixelCoord(pixelCoord),
              numPoint(n),
              signRho(signRho),
              signR3(signR3),
              _verbose(verbose),
              matP(Eigen::MatrixXf::Zero(2 * n, 12)){

    }
    ~SingleCamera() = default;  // destructor

    void composeMatP();

    void performSVDMatP();

    void solveIntrinsicAndExtrinsic();

    void selfCheck(const Eigen::MatrixXf &w_check, const Eigen::MatrixXf &c_check);

private:
    bool _verbose;  // for debug
    Eigen::MatrixXf worldCoord;
    Eigen::MatrixXf pixelCoord;
    size_t numPoint;
    float signRho{1.0};
    float signR3{1.0};

    Eigen::MatrixXf matP;  // matrix build up with world coordinate system and corresponding pixels. size: 2n x 12
    Eigen::MatrixXf matM{Eigen::MatrixXf::Zero(3, 4)};;
    Eigen::MatrixXf matA{Eigen::MatrixXf::Zero(3, 3)};
    Eigen::MatrixXf vecB{Eigen::VectorXf::Zero(3, 1)};
    Eigen::MatrixXf matK{Eigen::MatrixXf::Zero(3, 3)};
    Eigen::MatrixXf matR{Eigen::MatrixXf::Zero(3, 3)};
    Eigen::MatrixXf vecT{Eigen::VectorXf::Zero(3, 1)};
    Eigen::MatrixXf matExtrinsic{Eigen::MatrixXf::Zero(3, 4)};

    float calcTheta(const Eigen::Vector3f &, const Eigen::Vector3f &, const Eigen::Vector3f &);

};