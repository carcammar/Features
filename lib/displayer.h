#ifndef DISPLAYER_H
#define DISPLAYER_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

#include<pangolin/pangolin.h>

class SLAM;
#include "slam.h"


class Displayer
{
public:
    Displayer(SLAM *pSlam);
    void Run();
private:
    SLAM *mpSlam;
    cv::Mat mIm;
    std::vector<cv::KeyPoint> vPointsToDraw;
    std::vector<Point*> v3DPointsToDraw;
    std::vector<KeyFrame*> vpKFsToDraw;

    double mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

};

#endif // DISPLAYER_H
