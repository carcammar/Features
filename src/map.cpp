#include "map.h"

Map::Map(bool bLocal): mbLocalMap(bLocal), nPoints(0), nKFs(0)
{

}

Map::Map(std::vector<Point*> vpPoints, std::vector<KeyFrame*> vpKFs, bool bLocalMap): mbLocalMap(bLocalMap), nPoints(0), nKFs(0)
{
    mlpPoints.clear();
    mlpKFs.clear();


    // TODO Maybe resize is faster than push each one
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        mlpPoints.push_back(*itP);
        if (!bLocalMap)
        {

            (*itP)->SetMap(this);
        }
        else
            (*itP)->SetLocalMap(this);
        nPoints++;
    }
    for(std::vector<KeyFrame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        mlpKFs.push_back(*itKF);
        if (!bLocalMap)
        {
            (*itKF)->SetMap(this);
        }
        else
            (*itKF)->SetLocalMap(this);
        nKFs++;
    }
}

void Map::TriangulatePoints(Eigen::Matrix<float,3,3> K, Eigen::Matrix<float,4,4> Tcw1, Eigen::Matrix<float,4,4> Tcw2, std::vector<cv::Point2f> vP1, std::vector<cv::Point2f> vP2, std::vector<Eigen::Vector3f> &v3DP)
{
    /*Eigen::Matrix<float,4,3> A;
    Eigen::Matrix<float,4,1> b;*/

    Eigen::MatrixXf A;
    Eigen::MatrixXf b;
    A = Eigen::MatrixXf::Zero(4,3);
    b = Eigen::VectorXf::Zero(4);

    Eigen::Vector3f x;

    Eigen::Matrix<float,2,3> Kp;
    Kp = K.topLeftCorner(2,3);
    Eigen::Matrix<float,2,3> KpRcw1, KpRcw2;
    KpRcw1 = Kp*Tcw1.topLeftCorner(3,3);
    KpRcw2 = Kp*Tcw2.topLeftCorner(3,3);
    Eigen::Matrix<float,2,1> Kptcw1, Kptcw2;
    Kptcw1 = Kp*Tcw1.topRightCorner(3,1);
    Kptcw2 = Kp*Tcw2.topRightCorner(3,1);

    v3DP.clear();
    v3DP.reserve(vP1.size());

    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    // svd.setThreshold(1e-14);

    for(std::vector<cv::Point2f>::iterator itP1 = vP1.begin(), itP2 = vP2.begin(); itP1 != vP1.end(); itP1++, itP2++)
    {
        // TODO add some checking here to detect outliers
        // A
        A(0,0) = (*itP1).x*Tcw1(2,0)-KpRcw1(0,0);
        A(0,1) = (*itP1).x*Tcw1(2,1)-KpRcw1(0,1);
        A(0,2) = (*itP1).x*Tcw1(2,2)-KpRcw1(0,2);

        A(1,0) = (*itP1).y*Tcw1(2,0)-KpRcw1(1,0);
        A(1,1) = (*itP1).y*Tcw1(2,1)-KpRcw1(1,1);
        A(1,2) = (*itP1).y*Tcw1(2,2)-KpRcw1(1,2);

        A(2,0) = (*itP2).x*Tcw2(2,0)-KpRcw2(0,0);
        A(2,1) = (*itP2).x*Tcw2(2,1)-KpRcw2(0,1);
        A(2,2) = (*itP2).x*Tcw2(2,2)-KpRcw2(0,2);

        A(3,0) = (*itP2).y*Tcw2(2,0)-KpRcw2(1,0);
        A(3,1) = (*itP2).y*Tcw2(2,1)-KpRcw2(1,1);
        A(3,2) = (*itP2).y*Tcw2(2,2)-KpRcw2(1,2);

        // b
        b(0,0) = Kptcw1(0,0)-(*itP1).x*Tcw1(2,3);
        b(1,0) = Kptcw1(1,0)-(*itP1).y*Tcw1(2,3);

        b(2,0) = Kptcw2(0,0)-(*itP2).x*Tcw2(2,3);
        b(3,0) = Kptcw2(1,0)-(*itP2).y*Tcw2(2,3);

        // Solve by SVD
        /*std::cout << "pt1 = " << (*itP1) << std::endl;
        std::cout << "pt2 = " << (*itP2) << std::endl;
        std::cout << "A = " << A << std::endl;
        std::cout << "b = " << b << std::endl;*/

        svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        x = svd.solve(b);

        // std::cout << "X3d = " << x << std::endl;
        v3DP.push_back(x);
    }
}




void Map::IncludePoints(std::vector<Point*> vpPoints)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        mlpPoints.push_back(*itP);
        nPoints++;
    }
}

void Map::IncludePoint(Point* pPoint)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mlpPoints.push_back(pPoint);
    nPoints++;
}

void Map::IncludeKFs(std::vector<KeyFrame*> vpKFs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    for(std::vector<KeyFrame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        mlpKFs.push_back(*itKF);
        nKFs++;
    }
}

void Map::IncludeKF(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mlpKFs.push_back(pKF);
    nKFs++;
}

void Map::RemovePoint(Point* pPoint)
{
    // TODO MapMutex
    std::unique_lock<std::mutex> lock(mMutexMap);

}

void Map::RemoveKF(KeyFrame* pKF)
{
    // TODO MapMutex
    nKFs--;
    std::unique_lock<std::mutex> lock(mMutexMap);


}

std::vector<KeyFrame*> Map::GetKFs()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::vector<KeyFrame*> vpKFs;
    vpKFs.reserve(mlpKFs.size());
    for(std::list<KeyFrame*>::iterator itKF = mlpKFs.begin(); itKF != mlpKFs.end(); itKF++)
        vpKFs.push_back(*itKF);
    return vpKFs;
}
std::vector<Point*> Map::GetPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::vector<Point*> vpPoints;
    vpPoints.reserve(mlpPoints.size());
    for(std::list<Point*>::iterator itP = mlpPoints.begin(); itP != mlpPoints.end(); itP++)
        vpPoints.push_back(*itP);
    return vpPoints;
}

unsigned int Map::GetNumberPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return nPoints;
}

unsigned int Map::GetNumberKFs()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return nKFs;
}

void Map::GetStatus()
{
    std::cout << "Points in map: " << nPoints << "/" << mlpPoints.size() << std::endl;
    std::cout << "KF in map: " << nKFs << "/" << mlpKFs.size() << std::endl;
}
