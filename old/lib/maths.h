#ifndef MATHS_H
#define MATHS_H

#include <Eigen/Dense>
#include <cmath>

class Maths
{
public:
    static Eigen::Matrix<float,3,3> Skew(Eigen::Vector3f vec);
    static Eigen::Matrix<float,3,3> Skew(float phi1, float phi2, float phi3);
    static Eigen::Vector3f Unskew(Eigen::Matrix<float,3,3> mat);

    static Eigen::Matrix<float,3,3> ExpSO3(Eigen::Vector3f phi);
    static Eigen::Matrix<float,3,3> ExpSO3(double phi1, double phi2, double phi3);
    static Eigen::Vector3f LogSO3(Eigen::Matrix<float,3,3> R);

    static Eigen::Matrix<float,4,4> ExpSE3(Eigen::Vector3f phi);
    static Eigen::Matrix<float,6,1> LogSE3(Eigen::Matrix<float,4,4> T);
    static Eigen::Matrix<float,4,4> InvSE3(Eigen::Matrix<float,4,4> T);
};

#endif // MATHS_H
