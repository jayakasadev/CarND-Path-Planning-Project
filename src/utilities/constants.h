#include "map.h"

#ifndef CONSTANTS_H
#define CONSTANTS_H

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

// fixed speed limit
const double speed_limit = 49.99;

const short spacing = 30; // spacing in m between s way points

const float speed_adjust_rate = .224;

const float time_interval = .02;

const float time_period = 2.0;

const float mph_2_mps = 2.23694;

const short buffer = 5;

const float search_field = 2.5;

const float max_acceleration = 10;

const float max_jerk = 50;

const int num_points = time_period / time_interval;

const long time_diff = 5000; // if i do not see a vehicle for more than 5 seconds I forget about it

const map_data mapData = map_data::getInstance();

#endif //CONSTANTS_H
