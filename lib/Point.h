#ifndef POINT_H
#define POINT_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <lib/keyframe.h>

class Map;

class Point
{
public:
    Eigen::Vector3f mr; // Global cordinates
    std::map<KeyFrame*, Eigen::Vector2f> mmpKFObs; // map from pointer to KeyFrames to Observations
    Point();
    Point(unsigned int id, Eigen::Vector3f mean, float std);

    unsigned int mId;

    Map *mpMap;
    Map *mpLocalMap;

    double sigma; // Related with scale of extracted point
};

#endif // POINT_H
