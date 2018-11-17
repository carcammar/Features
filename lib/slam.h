#ifndef SLAM_H
#define SLAM_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <experimental/filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <mutex>

class Displayer;

#include "map.h"
#include "frame.h"
#include "displayer.h"

class SLAM
{
public:
    enum state{
        NOT_INITIALIZED = 1,
        OK = 2};

    SLAM();
    SLAM(std::string path_to_data, std::string path_to_calibration);
    void Run();

    bool bStop; // If system is requested to stop
    bool bDispImage; // Image to be displayed

    // KeyPoint extractors
    int nFeat;
    cv::Ptr<cv::Feature2D> mpFeatExt;
    cv::Ptr<cv::DescriptorMatcher> mpMatcher;

    // Pointers to maps
    Map *mpMap;
    Map *mpLocalMap;


    bool IsNewImage();
    cv::Mat GetImageToDraw();
    std::vector<cv::KeyPoint> GetPointToDraw();
private:
    void ReadCalibration();
    void GrabImage();
    void Initialize();
    void Track();

    // Camera matrix
    float fx;
    float fy;
    float cx;
    float cy;
    Eigen::Matrix3f mK;
    cv::Mat mcvK;

    // Distortion coefficients
    std::vector<float> mvDistCoeffs;
    float k1;
    float k2;
    float p1;
    float p2;

    unsigned int h;
    unsigned int w;
    float fps;

    std::vector<float> mvTimeStamps;
    std::vector<std::string> mvstrTimeStamps;

    // Pointer to current frame
    unsigned int nextKFId;
    unsigned int nextFId;
    unsigned int nextPointId;

    Frame *mpLastFrame; // TODO do not use pointers
    Frame *mpCurrFrame;
    KeyFrame *mpCurrKF;
    std::list<Frame*> mlpFrames; // TODO erase this attribute

    // Displayer object and parallel thread for its execution
    Displayer *mpDisplay;
    std::thread *mptDisplayer;

    // Current image to be shown
    cv::Mat mCurrIm;
    std::mutex mMutexIm;

    // For initialization
    Frame *mpFirstFrame;
    const int minInliers;


    state mState;
};

#endif // SLAM_H
