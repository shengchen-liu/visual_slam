#include "essential_matrix.h"

#include <complex>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d> &points,
                                   std::vector<Eigen::Vector2d> *normed_points,
                                   Eigen::Matrix3d *matrix) {
  // Calculate centroid
  Eigen::Vector2d centroid(0, 0);
  for (const Eigen::Vector2d &point: points) {
    centroid += point;
  }
  centroid /= points.size();

  // Root mean square error to centroid of all points
  double rms_mean_dist = 0;
  for (const Eigen::Vector2d &point: points) {
    rms_mean_dist += (point - centroid).squaredNorm();
  }
  rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

  // Compose normalization matrix
  const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
  *matrix << norm_factor, 0, -norm_factor * centroid(0), 0, norm_factor,
          -norm_factor * centroid(1), 0, 0, 1;

  // Apply normalization matrix
  normed_points->resize(points.size());

  const double M_00 = (*matrix)(0, 0);
  const double M_01 = (*matrix)(0, 1);
  const double M_02 = (*matrix)(0, 2);
  const double M_10 = (*matrix)(1, 0);
  const double M_11 = (*matrix)(1, 1);
  const double M_12 = (*matrix)(1, 2);
  const double M_20 = (*matrix)(2, 0);
  const double M_21 = (*matrix)(2, 1);
  const double M_22 = (*matrix)(2, 2);

  for (size_t i = 0; i < points.size(); ++i) {
    const double p_0 = points[i](0);
    const double p_1 = points[i](1);

    const double np_0 = M_00 * p_0 + M_01 * p_1 + M_02;
    const double np_1 = M_10 * p_0 + M_11 * p_1 + M_12;
    const double np_2 = M_20 * p_0 + M_21 * p_1 + M_22;

    const double inv_np_2 = 1.0 / np_2;
    (*normed_points)[i](0) = np_0 * inv_np_2;
    (*normed_points)[i](1) = np_1 * inv_np_2;
  }
}

Eigen::Matrix3d EssentialMatrixEightPointEstimate(const std::vector<Eigen::Vector2d> &points1,
                                                  const std::vector<Eigen::Vector2d> &points2) {

  // Center and normalize image points for better numerical stability.
  std::vector<Eigen::Vector2d> normed_points1;
  std::vector<Eigen::Vector2d> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  // Setup homogeneous linear equation as x2' * F * x1 = 0.
  Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
  for (size_t i = 0; i < points1.size(); ++i) {
    cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
    cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
    cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
  }

  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
          cmatrix, Eigen::ComputeFullV);
  const Eigen::VectorXd ematrix_nullspace = cmatrix_svd.matrixV().col(8);
  const Eigen::Map<const Eigen::Matrix3d> ematrix_t(ematrix_nullspace.data());

  // De-normalize to image points.
  const Eigen::Matrix3d E_raw = points2_norm_matrix.transpose() *
                                ematrix_t.transpose() * points1_norm_matrix;

  // Enforcing the internal constraint that two singular values must be equal
  // and one must be zero.
  Eigen::JacobiSVD<Eigen::Matrix3d> E_raw_svd(
          E_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = E_raw_svd.singularValues();
  singular_values(0) = (singular_values(0) + singular_values(1)) / 2.0;
  singular_values(1) = singular_values(0);
  singular_values(2) = 0.0;
  const Eigen::Matrix3d E = E_raw_svd.matrixU() * singular_values.asDiagonal() *
                            E_raw_svd.matrixV().transpose();

  return E;
}

Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d &vector) {
  Eigen::Matrix3d matrix;
  matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
          vector(0), 0;
  return matrix;
}

Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d &R,
                                        const Eigen::Vector3d &t) {
  Eigen::Matrix3d E;

  E = CrossProductMatrix(t.normalized()) * R;
  return E;
}

