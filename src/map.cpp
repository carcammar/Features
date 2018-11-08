#include "map.h"

Map::Map(): mbLocalMap(false), nPoints(0), nKFs(0)
{

}

Map::Map(std::vector<Point*> vpPoints, std::vector<Frame*> vpKFs, bool bLocalMap): mbLocalMap(bLocalMap), nPoints(0), nKFs(0)
{
    mlpPoints.clear();
    mlpKFs.clear();


    // TODO Maybe resize is faster than push each one
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        mlpPoints.push_back(*itP);
        if (!bLocalMap)
        {

            (*itP)->SetMap(this);
        }
        else
            (*itP)->SetLocalMap(this);
        nPoints++;
    }
    for(std::vector<Frame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        mlpKFs.push_back(*itKF);
        if (!bLocalMap)
        {
            (*itKF)->SetMap(this);
        }
        else
            (*itKF)->SetLocalMap(this);
        nKFs++;
    }
}


void Map::IncludePoints(std::vector<Point*> vpPoints)
{
    // TODO MapMutex
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        mlpPoints.push_back(*itP);
        nPoints++;
    }
}

void Map::IncludePoints(Point* pPoint)
{
    // TODO MapMutex
    mlpPoints.push_back(pPoint);
    nPoints++;
}

void Map::IncludeKFs(std::vector<Frame*> vpKFs)
{
    // TODO MapMutex
    for(std::vector<Frame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        mlpKFs.push_back(*itKF);
        nKFs++;
    }
}

void Map::IncludeKFs(Frame* pKF)
{
    // TODO MapMutex
    mlpKFs.push_back(pKF);
    nKFs++;
}

void Map::RemovePoint(Point* pPoint)
{
    // TODO MapMutex

}

void Map::RemoveKF(Frame* pFrame)
{
    // TODO MapMutex

}

std::vector<Frame*> Map::GetKFs()
{
    // TODO MapMutex
    std::vector<Frame*> vpKFs;
    vpKFs.reserve(mlpKFs.size());
    for(std::list<Frame*>::iterator itKF = mlpKFs.begin(); itKF != mlpKFs.end(); itKF++)
        vpKFs.push_back(*itKF);
    return vpKFs;
}
std::vector<Point*> Map::GetPoints()
{
    // TODO MapMutex
    std::vector<Point*> vpPoints;
    vpPoints.reserve(mlpPoints.size());
    for(std::list<Point*>::iterator itP = mlpPoints.begin(); itP != mlpPoints.end(); itP++)
        vpPoints.push_back(*itP);
    return vpPoints;
}

unsigned int Map::GetNumberPoints()
{
    // TODO MapMutex
    return nPoints;
}

unsigned int Map::GetNumberKFs()
{
    // TODO MapMutex
    return nKFs;
}
