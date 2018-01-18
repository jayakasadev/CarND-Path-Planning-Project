//
// Created by jay on 1/8/18.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_CONSTANTS_H
#define PATH_PLANNING_SENSOR_FUSION_CONSTANTS_H

// for sensor_fusion

#include "behavior_constants.h"

const float search_radius = 5; // search radius of 5m

const float growth_rate = (spacing - search_radius) / spacing; // the radius grows the faster i go

const short time_limit = 500; // forget the car if I have not seen it in .5 second

#endif //PATH_PLANNING_SENSOR_FUSION_CONSTANTS_H
