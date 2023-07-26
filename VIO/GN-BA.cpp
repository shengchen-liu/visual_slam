#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv)
{

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d
    ifstream infile3d(p3d_file);
    double x, y, z;
    while (infile3d >> x >> y >> z)
    {
        p3d.push_back(Eigen::Vector3d(x, y, z));
    }

    ifstream infile2d(p2d_file);
    while (infile2d >> x >> y)
    {
        p2d.push_back(Eigen::Vector2d(x, y));
    }
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    std::cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++)
    {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++)
        {
            // compute cost for p3d[I] and p2d[I]
            Eigen::Vector3d pc = T_esti * p3d[i];
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            Eigen::Vector2d e = p2d[i] - proj;
            cost += e.squaredNorm();

            // compute jacobian
            Matrix<double, 2, 6> J;
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            J << -fx * inv_z,
                0,
                fx * pc[0] * inv_z2,
                fx * pc[0] * pc[1] * inv_z2,
                -fx - fx * pc[0] * pc[0] * inv_z2,
                fx * pc[1] * inv_z,
                0,
                -fy * inv_z,
                fy * pc[1] * inv_z2,
                fy + fy * pc[1] * pc[1] * inv_z2,
                -fy * pc[0] * pc[1] * inv_z2,
                -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        dx = H.ldlt().solve(b);

        if (isnan(dx[0]))
        {
            std::cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            // cost increase, update is not good
            std::cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        T_esti = Sophus::SE3d::exp(dx) * T_esti;

        lastCost = cost;

        std::cout << "iteration " << iter << " cost=" << std::cout.precision(12) << cost << endl;
    }

    std::cout << "estimated pose: \n"
              << T_esti.matrix() << endl;
    return 0;
}
