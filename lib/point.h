#ifndef POINT_H
#define POINT_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <frame.h>

class Map;

class Point
{
public:
    Eigen::Vector3f mr; // Global cordinates
    std::map<Frame*, Eigen::Vector2f> mmpKFObs; // map from pointer to Frames to Observations
    Point();
    Point(unsigned int id, Eigen::Vector3f mean, float std);

    unsigned int mId;

    double sigma; // Related with scale of extracted point

    void SetMap(Map *map);
    void SetLocalMap(Map *map);

    void SetPosition(Eigen::Vector3f pos);

private:

    // TODO declare mr as private

    // Map information
    Map *mpMap;
    Map *mpLocalMap;
};

#endif // POINT_H
