#ifndef CERESTYPES_H
#define CERESTYPES_H

#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


#include "keyframe.h"
#include "Point.h"
#include "maths.h"

namespace cerestypes{

struct ReprojectionError {
    // undistorted observation!
    ReprojectionError(double _u, double _v)
        : u_obs(_u), v_obs(_v) {}

    template <typename T>
    bool operator()(const T* const camera, // (phi, t)
                    const T* const point, // r
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation from world to camera reference
        // camera[3,4,5] are the translation from world to camera reference
        // point[0,1,2] are the world coordinates of point to be reprojected

        T cPoint[3]; // Point in camera coordinates
        ceres::AngleAxisRotatePoint(camera, point, cPoint);

        // camera[3,4,5] are the translation.
        cPoint[0] += camera[3];
        cPoint[1] += camera[4];
        cPoint[2] += camera[5];

        /*Eigen::Vector3f phi, aux1, aux2;
        phi << static_cast<float>(camera[0]), static_cast<float>(camera[1]), static_cast<float>(camera[2]);
        aux1 << static_cast<float>(point[0]), static_cast<float>(point[1]), static_cast<float>(point[2]);
        aux2 << static_cast<float>(camera[3]), static_cast<float>(camera[4]), static_cast<float>(camera[5]);
        phi = Maths::ExpSE3(phi)*aux1 + aux2;
        std::cout << " Point: " << phi.transpose() << std::endl;*/

        /*T R1[9];
        ceres::AngleAxisToRotationMatrix(camera, R1);
        // Eigen::Matrix3f R2 = Maths::ExpSO3(camera[0],camera[1],camera[2]);
        std::cout << " R1 = " << R1[0] << ", " << R1[1] << ", " << R1[2] << ", " << R1[3] << ", " << R1[4] << ", " << R1[5] << ", " << R1[6] << ", " << R1[7] << ", " << R1[8] << std::endl; //<< "R2 = " << R2 << std::endl;
        std::cout << " phi = " << camera[0] << ", " << camera[1] << ", " << camera[2] << std::endl;
        T dist_1 = sqrt((camera[3]-point[0])*(camera[3]-point[0]) +
                (camera[4]-point[1])*(camera[4]-point[1]) +
                (camera[5]-point[2])*(camera[5]-point[2]));

        T dist_2 = sqrt(cPoint[0]*cPoint[0] + cPoint[1]*cPoint[1] + cPoint[2]*cPoint[2]);*/

        // std::cout << "(" << u_obs << ", " << v_obs << ")   cam-point: " << dist_1 << ", " << dist_2 << std::endl;


        // Project point into the image plane
        T xp = cPoint[0] / cPoint[2];
        T yp = cPoint[1] / cPoint[2];
        T predicted_x = fx*xp + cx;
        T predicted_y = fy*yp + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = sqrt_sigma_1*(predicted_x - u_obs);
        residuals[1] = sqrt_sigma_1*(predicted_y - v_obs);

        // std::cout << "error: " << residuals[0] << " / " << residuals[1] << std::endl;
        return true;
    }

    // Construct cost function from observation, KF and Point
    static ceres::CostFunction* Create(const double observed_x, const double observed_y, Point *pPoint, KeyFrame* pKF) {
        ReprojectionError *pRepError = new ReprojectionError(observed_x, observed_y);
        pRepError->fx = static_cast<double>(pKF->fx);
        pRepError->fy = static_cast<double>(pKF->fy);
        pRepError->cx = static_cast<double>(pKF->cx);
        pRepError->cy = static_cast<double>(pKF->cy);
        pRepError->sqrt_sigma_1 = sqrt(1/pPoint->sigma); // TODO too expensive this operation
        return(new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(pRepError));
    }

    static ceres::CostFunction* Create(const Eigen::Vector2f observed_xy, Point *pPoint, KeyFrame* pKF) {
        ReprojectionError *pRepError = new ReprojectionError(static_cast<double>(observed_xy(0)), static_cast<double>(observed_xy(1)));
        pRepError->fx = static_cast<double>(pKF->fx);
        pRepError->fy = static_cast<double>(pKF->fy);
        pRepError->cx = static_cast<double>(pKF->cx);
        pRepError->cy = static_cast<double>(pKF->cy);
        pRepError->sqrt_sigma_1 = sqrt(1/pPoint->sigma); // TODO too expensive this operation
        return(new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(pRepError));
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                    new ReprojectionError(observed_x, observed_y)));
    }

    // Undistorted observation
    double u_obs;
    double v_obs;

    // Camera parameters (no distortion)
    double fx;
    double fy;
    double cx;
    double cy;

    // Covariance of
    double sqrt_sigma_1;

};

}
#endif // CERESTYPES_H
