#ifndef MAP_H
#define MAP_H

#include <vector>

#include "Point.h"
#include "keyframe.h"
#include <iterator>

class Map
{
public:
    Map();
    Map(std::vector<Point*> vpPoints, std::vector<KeyFrame*> vpKFs, bool bLocalMap = false);

    void IncludePoints(std::vector<Point*> vpPoints);
    void IncludePoints(Point* pPoint);

    void IncludeKFs(std::vector<KeyFrame*> vpKFs);
    void IncludeKFs(KeyFrame* pKF);

    std::vector<Point*> mvpPoints;
    std::vector<KeyFrame*> mvpKFs;

private:

};

#endif // MAP_H
