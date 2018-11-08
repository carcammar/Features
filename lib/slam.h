#ifndef SLAM_H
#define SLAM_H


#include "map.h"
#include "frame.h"

class SLAM
{
public:
    SLAM();
    Map *mpMap;

private:
    void ReadCalibration();
    void GrabImage();
    void Run();
    void Initialize();

    Eigen::Matrix3f mK;
    unsigned int h;
    unsigned int w;

    Frame *mpLastFrame;
};

#endif // SLAM_H
