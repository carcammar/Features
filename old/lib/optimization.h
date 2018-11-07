#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "ceres/ceres.h"

#include "map.h"
#include "cerestypes.h"
#include "maths.h"

class Optimization
{
public:
    Optimization();

    static bool visualBA(Map* pMap);


};

#endif // OPTIMIZATION_H
