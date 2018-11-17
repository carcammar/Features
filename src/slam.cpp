#include "slam.h"

SLAM::SLAM(): bStop(false), bDispImage(false), mState(NOT_INITIALIZED), minInliers(50)
{

}

SLAM::SLAM(std::string path_to_data, std::string path_to_calibration): bStop(false), bDispImage(false), mState(NOT_INITIALIZED), minInliers(50)
{
    // 0. Get data
    // TODO do not suppose that they are named as 1,2,... N
    mvstrTimeStamps.reserve(12);
    mvTimeStamps.reserve(12);
    for(int i = 1; i<= 12; i++)
    {
        mvTimeStamps.push_back(static_cast<float>(i));
        mvstrTimeStamps.push_back(path_to_data + "/" + std::to_string(i) + std::string(".png"));
    }

    // 1. Get calibration parameters
    cv::FileStorage storage(path_to_calibration, cv::FileStorage::READ);
    // Camera matrix
    storage["Camera.fx"] >> fx;
    storage["Camera.fy"] >> fy;
    storage["Camera.cx"] >> cx;
    storage["Camera.cy"] >> cy;
    mK = Eigen::MatrixXf::Zero(3,3);
    mK(0,0) = fx;
    mK(0,2) = cx;
    mK(1,1) = fy;
    mK(1,2) = cy;
    mK(2,2) = 1.0f;
    mcvK = cv::Mat::zeros(3,3,CV_32F);
    mcvK.at<float>(0,0) = fx;
    mcvK.at<float>(1,1) = fy;
    mcvK.at<float>(0,2) = cx;
    mcvK.at<float>(1,2) = cy;
    mcvK.at<float>(2,2) = 1.0f;

    // Distortion parameters
    storage["Camera.k1"] >> k1;
    storage["Camera.k2"] >> k2;
    storage["Camera.p1"] >> p1;
    storage["Camera.p2"] >> p2;
    mvDistCoeffs.resize(4);
    mvDistCoeffs[0] = k1;
    mvDistCoeffs[1] = k2;
    mvDistCoeffs[2] = p1;
    mvDistCoeffs[3] = p2;

    // Image dimensions and frequency (TODO Load these values)
    w = 752;
    h = 480;

    storage["Camera.fps"] >> fps;
    storage.release();

    // 2. Create all necesary threads
    // 2.0 Create pointers to maps and frames
    nextKFId = 0;
    nextFId = 0;
    nextPointId = 0;
    mlpFrames.clear();
    mpFirstFrame = 0;
    mpLastFrame = 0;
    mpCurrFrame = 0;
    mpMap = new Map();
    mpLocalMap = new Map(true);

    // 2.1 Create and launch displayer
    mpDisplay = new Displayer(this);
    mptDisplayer = new std::thread(&Displayer::Run,mpDisplay);

    // 2.2 Create and Launch Local Mapping

    // 3. Create feature extractor TODO set these parameters from calibration file
    nFeat = 1000;
    mpFeatExt = cv::ORB::create(nFeat, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    /*cv::FlannBasedMatcher */mpMatcher = new cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    // mpMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

}

void SLAM::Run()
{
    // cv::namedWindow("Test", CV_WINDOW_AUTOSIZE);
    size_t i = 0;
    for(std::vector<std::string>::iterator itTs = mvstrTimeStamps.begin(); itTs != mvstrTimeStamps.end(); itTs++, i++)
    {
        mCurrIm = cv::imread((*itTs)); // Read the file

        // Do stuff here
        // 1. Create frame
        if (mpCurrFrame)
            mpLastFrame = mpCurrFrame;
        mpCurrFrame = new Frame(nextKFId++, mvTimeStamps[i], mK, mCurrIm,  mpFeatExt, mvDistCoeffs);
        mlpFrames.push_back(mpCurrFrame);
        bDispImage = true;

        if(mState == NOT_INITIALIZED)
        {
            Initialize();
        }
        else if(mState == OK)
        {
            Track();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
}

bool SLAM::IsNewImage()
{
    return bDispImage;
}

cv::Mat SLAM::GetImageToDraw()
{
    std::unique_lock<std::mutex> lock(mMutexIm);
    bDispImage = false;
    return mCurrIm.clone();
}

std::vector<cv::KeyPoint> SLAM::GetPointToDraw()
{
    // TODO here you should take matched points with those from map
    return mpCurrFrame->GetKPs();
}

void SLAM::Initialize()
{
    std::cout << "IN" << std::endl;
    if (!mpFirstFrame)
    {
        mpFirstFrame = mpCurrFrame;
        return;
    }

    // Find matches between current frame and first frame
    std::vector< std::vector<cv::DMatch> > knn_matches;
    cv::Mat Desc1 = mpFirstFrame->GetDescriptors();
    cv::Mat Desc2 = mpCurrFrame->GetDescriptors();
    mpMatcher->knnMatch(Desc1, Desc2, knn_matches, 2);

    std::cout << "knn_matches.size() " << knn_matches.size() << std::endl;
    const float ratio_thresh = 0.7f;

    std::vector< std::pair<unsigned int, unsigned int> > vPairMatchs;
    std::vector<cv::Point2f> vMatchedPoints1, vMatchedPoints2;
    vPairMatchs.reserve(knn_matches.size());
    vMatchedPoints1.reserve(knn_matches.size());
    vMatchedPoints2.reserve(knn_matches.size());

    std::vector<cv::Point2f> vP1 = mpFirstFrame->GetUndPoints();
    std::vector<cv::Point2f> vP2 = mpCurrFrame->GetUndPoints();

    // Check with Lowe test
    for (size_t i = 0; i < knn_matches.size(); i++)
        if (knn_matches[i].size()>=2)
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                // good_matches.push_back(knn_matches[i][0]);
                vPairMatchs.push_back(std::pair<unsigned int, unsigned int>(knn_matches[i][0].queryIdx , knn_matches[i][0].trainIdx));
                vMatchedPoints1.push_back(vP1[knn_matches[i][0].queryIdx]);
                vMatchedPoints2.push_back(vP2[knn_matches[i][0].trainIdx]);
            }

    // Find essential matrix and movement
    cv::Mat R;
    cv::Mat t;
    cv::Mat mask;

    cv::Mat Ess = cv::findEssentialMat(vMatchedPoints1, vMatchedPoints2, mcvK, CV_RANSAC, 0.999, 1.0, mask);

    int inliers = cv::recoverPose(Ess, vMatchedPoints1, vMatchedPoints2, mcvK, R, t, mask);

    // TODO remove outliers using mask
    Eigen::Matrix<float,4,4> Tcw1, Tcw2;
    Tcw1 = Eigen::Matrix<float,4,4>::Identity();
    Tcw2 = Eigen::Matrix<float,4,4>::Identity();
    Tcw2.topLeftCorner(3,3) = Maths::Cvmat2Eigmat(R.clone());
    Tcw2.topRightCorner(3,1) = Maths::Cvmat2Eigmat(t.clone());
    std::vector<Eigen::Vector3f> vPt3D;

    mpMap->TriangulatePoints(mK,Tcw1,Tcw2,vMatchedPoints1,vMatchedPoints2,vPt3D);

    int i=0;
    float meanDepth = 0.0f;
    inliers = 0;
    for(auto itPt = vPt3D.begin(); itPt != vPt3D.end(); itPt++, i++)
    {
        if(int(mask.at<uchar>(i,0)) == 1)
        {
            inliers++;
            // TODO remove too far points, only add points which are close
            meanDepth += (*itPt)(2);
        }
    }
    meanDepth = meanDepth/inliers;
    // TODO check this 7.0
    // Distance between two first cameras is set to one by default (baseline)
    if((meanDepth < 7.0f)&&(inliers > minInliers))
    {
        std::cout << "Good initialization with: " << std::endl;
        std::cout << "    mean depth: " << meanDepth << std::endl;
        std::cout << "    Points: " << inliers << std::endl;

        // Create first two KFs:
        mpFirstFrame->SetPose(Tcw1);
        mpCurrFrame->SetPose(Tcw2);
        KeyFrame* pKF1 = new KeyFrame(mpFirstFrame, nextKFId++);
        KeyFrame* pKF2 = new KeyFrame(mpCurrFrame, nextKFId++);

        Eigen::Vector2f obs1, obs2;

        // Add points to map and create observations
        Point *pPoint;
        i = 0;
        for(auto itPt = vPt3D.begin(); itPt != vPt3D.end(); itPt++, i++)
            if(int(mask.at<uchar>(i,0)) == 1)
            {
                pPoint = new Point(nextPointId++,(*itPt));
                pPoint->mNobs = 2;
                // TODO here Try to recover the righ scale for each feature
                pPoint->sigma = 1.0f;
                obs1 = Eigen::Vector2f(vMatchedPoints1[i].x, vMatchedPoints1[i].y);
                obs2 = Eigen::Vector2f(vMatchedPoints2[i].x, vMatchedPoints2[i].y);

                // Add observation to point object
                pPoint->AddObservation(pKF1, obs1, knn_matches[i][0].queryIdx);
                pPoint->AddObservation(pKF2, obs2, knn_matches[i][0].trainIdx);

                // Add observation to KF object
                pKF1->AddObservation(pPoint, obs1, knn_matches[i][0].queryIdx);
                pKF2->AddObservation(pPoint, obs2, knn_matches[i][0].trainIdx);

                // Add point to map
                mpMap->IncludePoint(pPoint);
            }
        // Add KF to map
        mpMap->IncludeKF(pKF1);
        mpMap->IncludeKF(pKF2);
        mpCurrKF = pKF2;

        std::cout << "Map well created!" << std::endl;
        mpMap->GetStatus();
        mState = OK;
    }
    else
    {
        std::cout << "not good initialization" << std::endl;
    }
}

void SLAM::Track()
{
    // 0. Match points with current KF
    std::vector< std::vector<cv::DMatch> > knn_matches;
    cv::Mat Desc1 = mpCurrKF->GetDescriptors();
    cv::Mat Desc2 = mpCurrFrame->GetDescriptors();
    mpMatcher->knnMatch(Desc1, Desc2, knn_matches, 2);

    // Apply ratio test to matched points
    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++)
        if (knn_matches[i].size()>=2)
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                // good_matches.push_back(knn_matches[i][0]);
                vPairMatchs.push_back(std::pair<unsigned int, unsigned int>(knn_matches[i][0].queryIdx , knn_matches[i][0].trainIdx));
                vMatchedPoints1.push_back(vP1[knn_matches[i][0].queryIdx]);
                vMatchedPoints2.push_back(vP2[knn_matches[i][0].trainIdx]);
            }

    // 1. Look for matched points which correspond with 3Dpoints

    // 2. With PnP solve for currentFrame pose

    // 3. Project all 3D points in map and search for descriptors in close region
}
