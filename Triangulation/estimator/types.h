#pragma once

#include <Eigen/Core>

namespace Eigen {

  typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;
  typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<uint8_t, 3, 1> Vector3ub;
  typedef Eigen::Matrix<uint8_t, 4, 1> Vector4ub;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

}  // namespace Eigen

