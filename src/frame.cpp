#include "frame.h"
#include "map.h"

Frame::Frame()
{

}

Frame::Frame(unsigned int id, Eigen::Matrix<float,4,4> Tcw, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w):
    mTcw(Tcw), mTs(ts), mK(K), mId(id), heigh(h), width(w)
{
    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);

    fx = mK(0,0);
    fy = mK(1,1);
    cx = mK(0,2);
    cy = mK(1,2);
}

Frame::Frame(Eigen::Matrix<float,6,1> mean, float stdAng, float stdTrans, unsigned int id, float ts, Eigen::Matrix<float,3,3> K, unsigned int h, unsigned int w):
    mK(K), mTs(ts), mId(id), heigh(h), width(w)
{
    Eigen::Vector3f phi(0.1f+stdAng*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f),
                        mean(1)+stdAng*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f),
                        mean(2)+stdAng*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f));
    mTcw(3,3) = 1.0f;
    mTcw.topLeftCorner(3,3) = Maths::ExpSO3(phi);
    mTcw(0,3) = stdTrans*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
    mTcw(1,3) = stdTrans*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
    mTcw(2,3) = stdTrans*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);

    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);

    fx = mK(0,0);
    fy = mK(1,1);
    cx = mK(0,2);
    cy = mK(1,2);
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
        cr = mTcw.topLeftCorner(3,3)*(*itP)->mr + mTcw.topRightCorner(3,1);
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
            mmpPObs[*itP] = u_obs;
            (*itP)->mmpKFObs[this] = u_obs;
        }
    }
}

void Frame::SetMap(Map *map)
{
    mpMap = map;
}

void Frame::SetLocalMap(Map *map)
{
    mpLocalMap = map;
}

void Frame::SetPose(Eigen::Matrix<float,4,4> Tcw)
{
    // TODO MutexPose
    mTcw = Tcw;
}

Eigen::Matrix<float,4,4> Frame::GetPose()
{
    return mTcw;
}
