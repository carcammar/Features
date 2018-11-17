#include "frame.h"
#include "map.h"

Frame::Frame()
{

}

Frame::Frame(unsigned int id, Eigen::Matrix<float,4,4> Tcw, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w):
    mId(id), mTs(ts), mNobs(0), mK(K), heigh(h), width(w), mTcw(Tcw)
{
    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);

    fx = mK(0,0);
    fy = mK(1,1);
    cx = mK(0,2);
    cy = mK(1,2);
}

Frame::Frame(unsigned int id, float ts, Eigen::Matrix<float,3,3> K, cv::Mat Im, cv::Ptr<cv::Feature2D> pFeat, std::vector<float> vDistCoeffs):
    mId(id), mTs(ts), mNobs(0), mK(K), mpFeatExt(pFeat), mvDistCoeffs(vDistCoeffs)
{
    // Extract points (TODO a grid to get more well distributed points)
    fx = mK(0,0);
    cx = mK(0,2);
    fy = mK(1,1);
    cy = mK(1,2);
    mpFeatExt->detectAndCompute(Im, cv::noArray(), mvKP, mDesc);

    // Undistort points
    mcvK = cv::Mat::zeros(3,3,CV_32F);
    mcvK.at<float>(0,0) = fx;
    mcvK.at<float>(1,1) = fy;
    mcvK.at<float>(0,2) = cx;
    mcvK.at<float>(1,2) = cy;
    mcvK.at<float>(2,2) = 1.0f;
    mvDistPoints.clear();
    mvpP = std::vector<Point*>(mvKP.size(), static_cast<Point*>(NULL));
    std::cout << "Size mvpP: " << mvpP.size() << std::endl;

    mvDistPoints.reserve(mvKP.size());
    for(std::vector<cv::KeyPoint>::iterator itKP = mvKP.begin(); itKP != mvKP.end(); itKP++)
    {
        mvDistPoints.push_back((*itKP).pt);
        // TODO maybe call inside this loop undistortion
    }
    cv::undistortPoints(mvDistPoints, mvUndPoints, mcvK, vDistCoeffs);
    for(std::vector<cv::Point2f>::iterator itP = mvUndPoints.begin(); itP != mvUndPoints.end(); itP++)
    {
        (*itP).x = (*itP).x*fx + cx;
        (*itP).y = (*itP).y*fy + cy;
    }
}


void Frame::ProjectMap(Map *map)
{
    // Project map points and search coincidences

    Eigen::Vector3f cr; // Point position relative to camera
    Eigen::Vector3f hu; // Homogeneous reprojection
    Eigen::Vector2f u;  // reprojection
    Eigen::Vector2f u_obs; // Observation
    bool bCoincidence;

    std::vector<Point*> vpPoints;
    vpPoints.reserve(map->GetNumberPoints());
    vpPoints = map->GetPoints();

    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        cr = mTcw.topLeftCorner(3,3)*(*itP)->GetPosition() + mTcw.topRightCorner(3,1);
        hu = mK*cr;
        u(0) = hu(0)/hu(2);
        u(1) = hu(1)/hu(2);

        // Search for coincidence in a region close to projection
        // TODO HERE, we should math with keypointin the image or do correlation search...
        if ((u(0)>0.0f)&&(u(0)<width)&&(u(1)>0.0f)&&(u(1)<heigh))
        {
            bCoincidence = true;
        }
        else
            bCoincidence = false;

        u_obs(0) = u(0);
        u_obs(1) = u(1);

        // Add observation if coincidence
        if (bCoincidence)
        {
            // TODO with add observation function
            /*mmpPObs[*itP] = u_obs;
            (*itP)->mmpFObs[this] = u_obs;*/
        }
    }
}


void Frame::SetPose(Eigen::Matrix<float,4,4> Tcw)
{
    std::unique_lock<std::mutex> lock(mMutexPosition);
    mTcw = Tcw;
}

Eigen::Matrix<float,4,4> Frame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPosition);
    return mTcw;
}

Eigen::Matrix<float,4,4> Frame::GetInvPose()
{
    std::unique_lock<std::mutex> lock(mMutexPosition);
    return Maths::InvSE3(mTcw);
}

std::vector<cv::KeyPoint> Frame::GetKPs()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mvKP;
}

cv::Mat Frame::GetDescriptors()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mDesc.clone();
}

std::vector<cv::Point2f> Frame::GetUndPoints()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mvUndPoints;
}

std::vector<cv::Point2f> Frame::GetDistPoints()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mvDistPoints;
}

std::map<Point*,unsigned int> Frame::GetpPIdx()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mmpPIdx;
}

std::map<Point*,Eigen::Vector2f> Frame::GetpPObs()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mmpPObs;
}

void Frame::AddObservation(Point *pPt, Eigen::Vector2f Obs, unsigned int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    mmpPObs.emplace(pPt,Obs);
    mmpPIdx.emplace(pPt,idx);
    mmIdxpP.emplace(idx,pPt); // TODO not truely necessary
    mvpP[idx] = pPt;
    mNobs++;
}

std::vector<Point*> Frame::GetObservedPoints()
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    return mvpP;
}


void Frame::SetMap(Map *map)
{
    mpMap = map;
}

void Frame::SetLocalMap(Map *map)
{
    mpLocalMap = map;
}

Map* Frame::GetMap()
{
    return mpMap;
}

Map* Frame::GetLocalMap()
{
    return mpLocalMap;
}

