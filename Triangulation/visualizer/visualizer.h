#include <pangolin/pangolin.h>

#include <Eigen/Core>

#include "camera.h"

void DrawFrames(std::vector<Eigen::Matrix4d> PoseTwc);

void DrawModel(Points ModelPoints, Lines ModelLines);

void DrawTwbFrames(std::vector<Eigen::Matrix4d> PoseTwb);

void DrawEstimatedModel(std::vector<Eigen::Vector3d> Points);