#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>

struct PoseData {
    double x, y, z;
};

const std::vector<PoseData> GARDEN_1_WAYPOINTS = {
    {-2.5, -2.0, 0.0},
    {-1.5, -3.0, 0.0},
    {-3.0, -5.0, 0.0},
    {-8.0, -5.0, 0.0},
    {-7.0, -3.5, 0.0},
    {-7.0, 0.0, 0.0},
    {-7.5, 2.0, 0.0}
};

#endif // WAYPOINTS_H