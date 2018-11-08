#ifndef Frame_H
#define Frame_H

#include <Eigen/Dense>
#include <map>
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <vector>

#include "maths.h"

class Map;
class Point; //Circular reference

class Frame
{
public:
    Frame();
    Frame(unsigned int id, Eigen::Matrix<float,4,4> Tcw, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w);
    Frame(Eigen::Matrix<float,6,1> mean, float stdAng, float stdTrans, unsigned int id, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w);

    void ProjectMap(Map *map);

    // General information
    unsigned int mId;

    float mTs; // timestamp

    // Observation information
    std::map<Point*,Eigen::Vector2f> mmpPObs;


    // Camera information
    Eigen::Matrix<float,3,3> mK;
    float fx;
    float fy;
    float cx;
    float cy;
    unsigned int heigh;
    unsigned int width;

    void SetMap(Map *map);
    void SetLocalMap(Map *map);

    void SetPose(Eigen::Matrix<float,4,4> Tcw);
    Eigen::Matrix<float,4,4> GetPose();
private:
    // Map information
    Map *mpMap;
    Map *mpLocalMap;

    Eigen::Matrix<float,4,4> mTcw;

};

#endif // Frame_H
