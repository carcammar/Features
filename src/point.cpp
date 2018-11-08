#include "point.h"

Point::Point()
{
    mr = Eigen::Vector3f(0.0f,.0f,.0f);

    mpMap = static_cast<Map*>(NULL);
    mpLocalMap = static_cast<Map*>(NULL);
}

Point::Point(unsigned int id, Eigen::Vector3f mean, float std): mId(id)
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
    // TODO PositionMutex
    mr = pos;
}
