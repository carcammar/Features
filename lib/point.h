#ifndef POINT_H
#define POINT_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <frame.h>
#include <keyframe.h>

class Map;

class Point
{
public:
    Point(unsigned int id, Eigen::Vector3f pos);
    Point(unsigned int id, Eigen::Vector3f mean, float std);

    unsigned int mId;
    unsigned int mNobs;

    double sigma; // Related with scale of extracted point

    void SetMap(Map *map);
    void SetLocalMap(Map *map);
    void SetPosition(Eigen::Vector3f pos);
    Eigen::Vector3f GetPosition();
    void AddObservation(KeyFrame* pKF, Eigen::Vector2f Obs, unsigned int idx);
    void RemoveObservation(KeyFrame *pKF);
    bool GetObservation(KeyFrame* pKF, Eigen::Vector2f &pt);
private:

    std::mutex mMutexPosition;
    std::mutex mMutexObservation;

    Eigen::Vector3f mr; // Global cordinates
    std::map<KeyFrame*, Eigen::Vector2f> mmpKFObs; // map from pointer to Frames to Observations
    std::map<KeyFrame*,int> mmpKFIdx; // index for observation of this point in corresponding KF

    // Map information
    Map *mpMap;
    Map *mpLocalMap;
};

#endif // POINT_H
