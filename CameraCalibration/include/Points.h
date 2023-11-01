#pragma once

#include <Eigen/Dense>

struct CalibrationPoints {
    // points for calibration

    int numPoints{0};
    Eigen::MatrixXf worldCoord;
    Eigen::MatrixXf pixelCoord;

    Eigen::MatrixXf worldCoordTest;
    Eigen::MatrixXf cameraCoordTest;

    CalibrationPoints() {

      // points for calibration
      Eigen::MatrixXf w_xz(4, 4);
      w_xz << 8, 0, 9, 1,
              8, 0, 1, 1,
              6, 0, 1, 1,
              6, 0, 9, 1;
      numPoints += w_xz.rows();

      Eigen::MatrixXf w_xy(4, 4);
      w_xy << 5, 1, 0, 1,
              5, 9, 0, 1,
              4, 9, 0, 1,
              4, 1, 0, 1;
      numPoints += w_xy.rows();

      Eigen::MatrixXf w_yz(4, 4);
      w_yz << 0, 4, 7, 1,
              0, 4, 3, 1,
              0, 8, 3, 1,
              0, 8, 7, 1;
      numPoints += w_yz.rows();

      worldCoord = Eigen::MatrixXf(numPoints, 4);
      worldCoord << w_xz, w_xy, w_yz;

      Eigen::MatrixXf c_xz(4, 2);
      c_xz << 275, 142,
              312, 454,
              382, 436,
              357, 134;

      Eigen::MatrixXf c_xy(4, 2);
      c_xy << 432, 473,
              612, 623,
              647, 606,
              464, 465;

      Eigen::MatrixXf c_yz(4, 2);
      c_yz << 654, 216,
              644, 368,
              761, 420,
              781, 246;

      pixelCoord = Eigen::MatrixXf(numPoints, 2);
      pixelCoord << c_xz, c_xy, c_yz;

      // points for test
      worldCoordTest = Eigen::MatrixXf(5, 4);
      worldCoordTest << 6, 0, 5, 1,
              3, 3, 0, 1,
              0, 4, 0, 1,
              0, 4, 4, 1,
              0, 0, 7, 1;
      cameraCoordTest = Eigen::MatrixXf(5, 2);
      cameraCoordTest << 369, 297,
              531, 484,
              640, 468,
              646, 333,
              556, 194;
    }

};
