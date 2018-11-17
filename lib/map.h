#ifndef MAP_H
#define MAP_H

#include <Eigen/SVD>
#include <Eigen/Jacobi>
#include <vector>
#include <iterator>
#include <list>

#include "point.h"
#include "keyframe.h"


class Map
{
public:
    Map(bool bLocal = false);
    Map(std::vector<Point*> vpPoints, std::vector<KeyFrame*> vpKFs, bool bLocalMap = false);

    void IncludePoints(std::vector<Point*> vpPoints);
    void IncludePoint(Point* pPoint);
    void RemovePoint(Point* pPoint);

    void IncludeKFs(std::vector<KeyFrame*> vpKFs);
    void IncludeKF(KeyFrame* pKF);
    void RemoveKF(KeyFrame* pKF);

    std::vector<KeyFrame*> GetKFs();
    std::vector<Point*> GetPoints();
    unsigned int GetNumberPoints();
    unsigned int GetNumberKFs();
    void TriangulatePoints(Eigen::Matrix<float,3,3> K, Eigen::Matrix<float,4,4> Tcw1, Eigen::Matrix<float,4,4> Tcw2, std::vector<cv::Point2f> vP1, std::vector<cv::Point2f> vP2, std::vector<Eigen::Vector3f> &v3DP);

    void GetStatus();


private:
    bool mbLocalMap;

    std::mutex mMutexMap;

    std::list<Point*> mlpPoints;
    std::list<KeyFrame*> mlpKFs;

    unsigned int nPoints;
    unsigned int nKFs;
};

#endif // MAP_H
