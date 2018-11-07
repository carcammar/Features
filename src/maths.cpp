#include "maths.h"

Eigen::Matrix<float,3,3> Maths::Skew(Eigen::Vector3f vec)
{
    Eigen::Matrix<float,3,3> mat = Eigen::MatrixXf::Zero(3,3);
    mat(0,1) = -vec(2);
    mat(1,0) = vec(2);
    mat(0,2) = vec(1);
    mat(2,0) = -vec(1);
    mat(1,2) = -vec(0);
    mat(2,1) = vec(0);

    return mat;
}

Eigen::Matrix<float,3,3> Maths::Skew(float phi1, float phi2, float phi3)
{
    Eigen::Matrix<float,3,3> mat = Eigen::MatrixXf::Zero(3,3);
    mat(0,1) = -phi3;
    mat(1,0) = phi3;
    mat(0,2) = phi2;
    mat(2,0) = -phi2;
    mat(1,2) = -phi1;
    mat(2,1) = phi1;

    return mat;
}

Eigen::Vector3f Maths::Unskew(Eigen::Matrix<float,3,3> mat)
{
    return Eigen::Vector3f(mat(2,1),mat(0,2),mat(1,0));
}


// For SO3
Eigen::Matrix<float,3,3> Maths::ExpSO3(Eigen::Vector3f phi)
{
    float normPhi2 = phi.transpose()*phi;
    float normPhi = std::sqrt(normPhi2);
    return Eigen::MatrixXf::Identity(3,3) + (std::sin(normPhi)/normPhi)*Skew(phi) + ((1-std::cos(normPhi))/normPhi2)*(Skew(phi)*Skew(phi));
}

Eigen::Matrix<float,3,3> Maths::ExpSO3(double phi1, double phi2, double phi3)
{
    float normPhi2 = static_cast<float>(phi1*phi1+phi2*phi2+phi3*phi3);
    float normPhi = sqrt(normPhi2);
    return Eigen::MatrixXf::Identity(3,3) + (std::sin(normPhi)/normPhi)*Skew(phi1, phi2, phi3) + ((1-std::cos(normPhi))/normPhi2)*(Skew(phi1, phi2, phi3)*Skew(phi1, phi2, phi3));
}

Eigen::Vector3f Maths::LogSO3(Eigen::Matrix<float,3,3> R)
{
    float phi = std::acos((R.trace()-1.0f)/2);
    return (phi/2/std::sin(phi))*Unskew(R-R.transpose());
}



// For SE3
Eigen::Matrix<float,4,4> Maths::ExpSE3(Eigen::Vector3f phi)
{
    // TODO IMPLEMENTATION
    return Eigen::MatrixXf::Identity(4,4);
}

Eigen::Matrix<float,6,1> Maths::LogSE3(Eigen::Matrix<float,4,4> T)
{
    // TODO IMPLEMENTATION
    return Eigen::MatrixXf::Zero(6,1);
}

Eigen::Matrix<float,4,4> Maths::InvSE3(Eigen::Matrix<float,4,4> T)
{
    Eigen::Matrix<float,4,4> mat;
    mat.topLeftCorner(3,3) = T.topLeftCorner(3,3).transpose();
    mat.topRightCorner(3,1) = -T.topLeftCorner(3,3).transpose()*T.topRightCorner(3,1);
    mat(3,0) = 0.f;
    mat(3,1) = 0.f;
    mat(3,2) = 0.f;
    mat(3,3) = 1.0f;
    return mat;
}
