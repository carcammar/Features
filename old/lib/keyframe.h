#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <Eigen/Dense>
#include <map>
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <vector>

#include "maths.h"

class Map;
class Point; //Circular reference

class KeyFrame
{
public:
    KeyFrame();
    KeyFrame(unsigned int id, Eigen::Matrix<float,4,4> Tcw, float ts, Eigen::Matrix<float,3,3> K, float h, float w);
    KeyFrame(Eigen::Matrix<float,6,1> mean, float stdAng, float stdTrans, unsigned int id, float ts, Eigen::Matrix<float,3,3> K, float h, float w);

    void ProjectMap(Map *map);

    // General information
    unsigned int mId;

    float mTs; // timestamp

    // Observation information
    std::map<Point*,Eigen::Vector2f> mmpPObs;

    // Map information
    Map *mpMap;
    Map *mpLocalMap;

    // Camera information
    Eigen::Matrix<float,4,4> mTcw;
    Eigen::Matrix<float,3,3> mK;
    float fx;
    float fy;
    float cx;
    float cy;
    unsigned int heigh;
    unsigned int width;

};

#endif // KEYFRAME_H
