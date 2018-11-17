#ifndef Frame_H
#define Frame_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <map>
#include <stdlib.h>
#include <iostream>
#include <iterator>
#include <vector>
#include <mutex>

#include "maths.h"

class Map;
class Point; //Circular reference


class Frame
{
public:
    Frame();
    Frame(unsigned int id, Eigen::Matrix<float,4,4> Tcw, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w);
    Frame(unsigned int id, float ts, Eigen::Matrix<float,3,3> K, cv::Mat Im, cv::Ptr<cv::Feature2D> pFeat, std::vector<float> vDistCoeffs);


    // Frame(Eigen::Matrix<float,6,1> mean, float stdAng, float stdTrans, unsigned int id, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w);

    void ProjectMap(Map *map);

    // General information
    unsigned int mId;
    float mTs; // timestamp
    unsigned int mNobs; // Number of observed points




    // Camera information
    Eigen::Matrix<float,3,3> mK;
    cv::Mat mcvK;
    float fx;
    float fy;
    float cx;
    float cy;

    std::vector<float> mvDistCoeffs;
    unsigned int heigh;
    unsigned int width;

    // Feature extractor
    cv::Ptr<cv::Feature2D> mpFeatExt;

    void SetMap(Map *map);
    void SetLocalMap(Map *map);

    void SetPose(Eigen::Matrix<float,4,4> Tcw);
    Eigen::Matrix<float,4,4> GetPose();
    Eigen::Matrix<float,4,4> GetInvPose();

    Map* GetMap();
    Map* GetLocalMap();
    std::vector<cv::KeyPoint> GetKPs();
    std::vector<cv::Point2f> GetUndPoints();
    std::vector<cv::Point2f> GetDistPoints();
    std::vector<Point*> GetObservedPoints();
    cv::Mat GetDescriptors();
    std::map<Point*,unsigned int> GetpPIdx();
    std::map<Point*,Eigen::Vector2f> GetpPObs();
    void AddObservation(Point *pPt, Eigen::Vector2f Obs, unsigned int idx);


protected:
    // mutex for reading frame values
    std::mutex mMutexPosition;
    std::mutex mMutexObservation;

    // Map information
    Map *mpMap;
    Map *mpLocalMap;

    Eigen::Matrix<float,4,4> mTcw;

    // All these 4 vectors and cv::Mat have the same dimension
    std::vector<cv::KeyPoint> mvKP;
    std::vector<cv::Point2f> mvUndPoints;
    std::vector<cv::Point2f> mvDistPoints;
    std::vector<Point*> mvpP;
    cv::Mat mDesc;

    // Observation information
    std::map<Point*,unsigned int> mmpPIdx; // Index of feaute from mvKP, mvUndPoints, mvDistPoints, mvpP and mDesc
    std::map<unsigned int ,Point*> mmIdxpP; // From index to 3d point
    std::map<Point*,Eigen::Vector2f> mmpPObs; // Undistorted position of 3dpoint in this frame
};

#endif // Frame_H
