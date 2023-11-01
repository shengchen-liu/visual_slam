#include <cfloat>
#include <iostream>
#include <Eigen/SVD>

#include "SingleCamera.h"

void SingleCamera::composeMatP() {
    // construct P matrix: get the form Pm = 0

    size_t numWorldCoords = worldCoord.rows();
    size_t numPixelCoords = pixelCoord.rows();

    assert(numWorldCoords == numPixelCoords);

    for (size_t idxRow = 0; idxRow < 2 * numPixelCoords; ++idxRow) {
        size_t idxPoint = idxRow / 2;
        Eigen::MatrixXf wp = worldCoord.row(idxPoint); // P_i
        Eigen::MatrixXf zeros(1, 4);
        zeros << 0.0, 0.0, 0.0, 0.0;

        if (idxRow % 2 == 0) {
          // P_i, 0, -u_i * Pi
          Eigen::MatrixXf cp = -pixelCoord(idxPoint, 0) * wp; // -u_i * P_i
            matP.row(idxRow) << wp, zeros, cp;
        }
        else {
          // 0, P_i, -v_i * Pi
            Eigen::MatrixXf cp = -pixelCoord(idxPoint, 1) * wp;
            matP.row(idxRow) << zeros, wp, cp;
        }
    }

    if (_verbose) {
        std::cout << "P matrix = " << std::endl << matP << std::endl;
    }
}

void SingleCamera::performSVDMatP() {
    /**
     * 1. Perform SVD
     * 2. Get V
     * 3. Use the last column of V
     * 4. Assign the last column to M
     **/
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(matP, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf  matV{svd.matrixV()};
    size_t numColMatV{static_cast<size_t>(matV.cols())};

    Eigen::VectorXf vecM(numColMatV);
    vecM << matV.col(numColMatV - 1);  // take the last column of V

    size_t numRowMatM{static_cast<size_t>(matM.rows())};
    size_t numColMatM{static_cast<size_t>(matM.cols())};
    matM = Eigen::MatrixXf::Map(vecM.data(), numColMatM, numRowMatM).transpose();

    // decompose matrix M into 3x3 matrix A and 3 x 1 vector b
    matA << matM.block(0, 0, 3, 3);
    vecB << matM.col(3);

    if (_verbose) {
        std::cout << "\nvecM = \n" << vecM << std::endl;
        std::cout << "\nM = \n" << matM << std::endl;
        std::cout << "\nA = \n" << matA << std::endl;
        std::cout << "\nb = \n" << vecB << std::endl;
    }

}

void SingleCamera::solveIntrinsicAndExtrinsic() {
    // solve for intrinsic and extrinsic parameters

    Eigen::Vector3f a1{matA.row(0)};
    Eigen::Vector3f a2{matA.row(1)};
    Eigen::Vector3f a3{matA.row(2)};

    if (_verbose) {
        std::cout << "\na1 = " << std::endl << a1 << std::endl;
        std::cout << "\na2 = " << std::endl << a2 << std::endl;
        std::cout << "\na3 = " << std::endl << a3 << std::endl;
    }

    // intrinsic parameters
    float rho{signRho / a3.norm()};
    float squareRho{static_cast<float>(pow(rho, 2))};
    float cx{squareRho * a1.dot(a3)};
    float cy{squareRho * a2.dot(a3)};
    float skewness{calcTheta(a1, a2, a3)};
    float sinTheta{static_cast<float>(sin(skewness))};
    float alpha{squareRho * a1.cross(a3).norm() * sinTheta};
    float beta{squareRho * a2.cross(a3).norm() * sinTheta};

    matK(0, 0) = alpha;
    matK(0, 1) = -alpha / tan(skewness);
    matK(0, 2) = cx;
    matK(1, 1) = beta / sinTheta;
    matK(1, 2) = cy;
    matK(2, 2) = 1.0;

    // extrinsic parameters
    Eigen::Vector3f cross23{a2.cross(a3)};
    Eigen::Vector3f r1{cross23 / cross23.norm()};
    Eigen::Vector3f r3{signR3 * a3 / a3.norm()};
    Eigen::Vector3f r2{r3.cross(r1)};

    matR << r1, r2, r3;
    matR.transposeInPlace();

    vecT = rho * matK.inverse() * vecB;

    Eigen::MatrixXf Rt(3, 4);

    matExtrinsic.block<3, 3>(0, 0) = matR;
    matExtrinsic.block<3, 1>(0, 3) = vecT;

    matM = matK * matExtrinsic;  // M = K * [R, t]

    if (_verbose) {
        std::cout << "\nK = \n" << matK << std::endl;
        std::cout << "\nR = \n" << matR << std::endl;
        std::cout << "\nt = \n" << vecT << std::endl;
        std::cout << "\n[R t] = \n" << matExtrinsic << std::endl;
        std::cout << "\nM = \n" << matM << std::endl;
    }

}

void SingleCamera::selfCheck(const Eigen::MatrixXf & worldCoordTest, const Eigen::MatrixXf & cameraCoordTest) {
    // check the correctness of M matrix

    size_t numTestPoints = worldCoordTest.rows();
    assert(numTestPoints == cameraCoordTest.rows());

    Eigen::MatrixXf homoPixelCoord(numTestPoints, 3);

    Eigen::MatrixXf pixelCoord(numTestPoints, 2);
    homoPixelCoord = (matM * worldCoordTest.transpose()).transpose();
    for (int i = 0; i < numTestPoints; ++i) {
        pixelCoord(i, 0) = homoPixelCoord(i, 0) / homoPixelCoord(i, 2);
        pixelCoord(i, 1) = homoPixelCoord(i, 1) / homoPixelCoord(i, 2);
    }

    std::cout << "\npixels given = \n"  << cameraCoordTest << std::endl;
    std::cout << "\npixels projected = \n"  << pixelCoord << std::endl;

    float average_err =  (cameraCoordTest - pixelCoord).cwiseAbs().sum() / (numTestPoints * 2);
    std::cout << "error = " << average_err << std::endl;
}

float SingleCamera::calcTheta(const Eigen::Vector3f & a1, const Eigen::Vector3f & a2, const Eigen::Vector3f & a3) {
    /**
     * Calculate the skewness angle, theta
     *
     * Input Parameters:
     * a1, a2, a3 -- the 1st, 2nd and 3rd rows of A matrix
     *
     * Return:
     * theta -- the skewness angle
     *
     * cos(theta) = (a1 x a3) * (a2 x a3) / |a1 x a3| * |a2 x a3|
     * where x and * refers to cross product and dot product
     *
     **/

    Eigen::Vector3f cross13{a1.cross(a3)};
    Eigen::Vector3f cross23{a2.cross(a3)};

    float cosTheta{-cross13.dot(cross23) / (cross13.norm() * cross23.norm())};
    float theta{static_cast<float>(acos(cosTheta))};

    return theta;
}
