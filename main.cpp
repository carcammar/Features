#include <Eigen/Dense>

#include <iterator>
#include <stdlib.h>
#include<iostream>
#include <vector>

#include "slam.h"
#include "point.h"
#include "frame.h"
#include "map.h"
#include "optimization.h"
#include "maths.h"





int main(int argc, char *argv[])
{
    std::cout << "Feature SLAM" << std::endl;
    SLAM slamSystem("/home/carlos/Desktop/sequences/test2/", "/home/carlos/Desktop/sequences/test2/euroc.yaml");
    slamSystem.Run();
    return 0;
}




/*int main(int argc, char *argv[])
{


    std::cout << "Test of Feature SLAM" << std::endl;
    const int Npoints = 100;
    const int NKF = 5;
    Eigen::Matrix<float,3,3> K = Eigen::MatrixXf::Identity(3,3);
    K(0,0) = 458.654f;
    K(1,1) = 457.296f;
    K(0,2) = 367.215f;
    K(1,2) = 248.375f;
    int heigh = 1000;
    int width = 1000;

    Map *pMap;

    // 0. Create Points
    Eigen::Vector3f meanPoint(.0f,0.f,5.0f);
    float stdPoint = 5.0f;

    std::vector<Point*> vpPoints(Npoints);

    unsigned int id = 0;
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        (*itP) = new  Point(id++, meanPoint, stdPoint);
    }


    // 1. Create KF
    Eigen::Matrix<float,6,1> meanCamera;
    meanCamera << 0.f, 0.f,0.f,0.f,0.f,0.f;
    float stdAngCam = 0.7f;
    float stdTransCam = 2.0f;

    std::vector<Frame*> vpKFs(NKF);
    float ts = 0.0f;
    id = 0;
    for(std::vector<Frame*>::iterator itCam = vpKFs.begin(); itCam != vpKFs.end(); itCam++)
    {
        (*itCam) = new  Frame(meanCamera, stdAngCam, stdTransCam, ts, id++, K, heigh, width);
        ts += 1.0f;
    }


    // 2. Create Map and add observations
    pMap = new Map(vpPoints, vpKFs);
    vpKFs = pMap->GetKFs();
    for(std::vector<Frame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        (*itKF)->ProjectMap(pMap);
    }

    for(unsigned int i = 0; i < vpKFs.size(); i++)
    {
        if (i==0)
            continue;
        Eigen::Matrix4f auxMat = vpKFs[i]->GetPose()*Maths::InvSE3(vpKFs[i-1]->GetPose());
        std::cout << " T_{i,i-1} " << std::endl << auxMat << std::endl;
        std::cout << "D_t = " << sqrt(auxMat(0,3)*auxMat(0,3)+auxMat(1,3)*auxMat(1,3)+auxMat(2,3)*auxMat(2,3)) << std::endl;
    }

    // 3. Perform BA
    Optimization::visualBA(pMap);

    vpKFs = pMap->GetKFs();
    for(unsigned int i = 0; i < vpKFs.size(); i++)
    {
        if (i==0)
            continue;
        Eigen::Matrix4f auxMat = vpKFs[i]->GetPose()*Maths::InvSE3(vpKFs[i-1]->GetPose());
        std::cout << " T_{i,i-1} " << std::endl << auxMat << std::endl;
        std::cout << "D_t = " << sqrt(auxMat(0,3)*auxMat(0,3)+auxMat(1,3)*auxMat(1,3)+auxMat(2,3)*auxMat(2,3)) << std::endl;

    }

    return 0;
}*/
