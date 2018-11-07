#include "map.h"

Map::Map()
{

}

Map::Map(std::vector<Point*> vpPoints, std::vector<KeyFrame*> vpKFs, bool bLocalMap)
{
    mvpPoints.clear();
    mvpKFs.clear();

    // TODO Maybe resize is faster than push each one
    for(std::vector<Point*>::iterator itP = vpPoints.begin(); itP != vpPoints.end(); itP++)
    {
        mvpPoints.push_back(*itP);
        if (!bLocalMap)
        {

            (*itP)->mpMap = this;
        }
        else
            (*itP)->mpLocalMap = this;
    }
    for(std::vector<KeyFrame*>::iterator itKF = vpKFs.begin(); itKF != vpKFs.end(); itKF++)
    {
        mvpKFs.push_back(*itKF);
        if (!bLocalMap)
        {
            (*itKF)->mpMap = this;
        }
        else
            (*itKF)->mpLocalMap = this;
    }
}


void Map::IncludePoints(std::vector<Point*> vpPoints)
{

}

void Map::IncludePoints(Point* pPoint)
{

}

void Map::IncludeKFs(std::vector<KeyFrame*> vpKFs)
{

}

void Map::IncludeKFs(KeyFrame* pKF)
{

}
