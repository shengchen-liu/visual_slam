#pragma once

#include <vector>

#include <Eigen/Core>

// Essential matrix estimator from corresponding normalized point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.


// The minimum number of samples needed to estimate a model.
static const int kMinNumSamples = 8;

// Estimate essential matrix solutions from  set of corresponding points.
//
// The number of corresponding points must be at least 8.
//
// @param points1  First set of corresponding points.
// @param points2  Second set of corresponding points.
Eigen::Matrix3d EssentialMatrixEightPointEstimate(const std::vector<Eigen::Vector2d>& points1,
                                                    const std::vector<Eigen::Vector2d>& points2);


// Compose essential matrix from relative camera poses.
//
// Assumes that first camera pose has projection matrix P = [I | 0], and
// pose of second camera is given as transformation from world to camera system.
//
// @param R             3x3 rotation matrix.
// @param t             3x1 translation vector.
//
// @return              3x3 essential matrix.
Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t);

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix);