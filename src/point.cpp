#include "point.h"

Point::Point(unsigned int id, Eigen::Vector3f pos): mId(id), mr(pos)
{
    mmpKFObs.clear(); // map from pointer to Frames to Observations
    mmpKFIdx.clear(); // inde

    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);
}

Point::Point(unsigned int id, Eigen::Vector3f mean, float std): mId(id), mNobs(0)
{
    mr = Eigen::Vector3f((static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f)*std + mean(0),
                         (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f)*std + mean(1),
                         (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f)*std + mean(2));

    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);

    sigma = 1.0; // pixels TODO CHANGE
}

void Point::SetMap(Map *map)
{
    mpMap = map;
}

void Point::SetLocalMap(Map *map)
{
    mpLocalMap = map;
}

void Point::SetPosition(Eigen::Vector3f pos)
{
    std::unique_lock<std::mutex> lock(mMutexPosition);
    mr = pos;
}

Eigen::Vector3f Point::GetPosition()
{
    std::unique_lock<std::mutex> lock(mMutexPosition);
    return mr;
}

void Point::AddObservation(KeyFrame* pKF, Eigen::Vector2f Obs, unsigned int idx)
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    mmpKFObs.emplace(pKF,Obs);
    mmpKFIdx.emplace(pKF,idx);
    mNobs++;
}

void Point::RemoveObservation(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    mmpKFObs.erase(pKF);
    mmpKFIdx.erase(pKF);
    mNobs--;
}

bool Point::GetObservation(KeyFrame* pKF, Eigen::Vector2f &pt)
{
    std::unique_lock<std::mutex> lock(mMutexObservation);
    std::map<KeyFrame*,Eigen::Vector2f>::iterator itMap;
    itMap = mmpKFObs.find(pKF);
    if (itMap == mmpKFObs.end())
        return false;
    else
    {
        pt = itMap->second;
        return true;
    }
}
