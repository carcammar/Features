#ifndef MAP_H
#define MAP_H

#include <vector>
#include <iterator>
#include <list>

#include "point.h"
#include "frame.h"


class Map
{
public:
    Map();
    Map(std::vector<Point*> vpPoints, std::vector<Frame*> vpKFs, bool bLocalMap = false);

    void IncludePoints(std::vector<Point*> vpPoints);
    void IncludePoints(Point* pPoint);
    void RemovePoint(Point* pPoint);

    void IncludeKFs(std::vector<Frame*> vpKFs);
    void IncludeKFs(Frame* pKF);
    void RemoveKF(Frame* pFrame);

    std::vector<Frame*> GetKFs();
    std::vector<Point*> GetPoints();
    unsigned int GetNumberPoints();
    unsigned int GetNumberKFs();



private:
    bool mbLocalMap;

    std::list<Point*> mlpPoints;
    std::list<Frame*> mlpKFs;

    unsigned int nPoints;
    unsigned int nKFs;
};

#endif // MAP_H
