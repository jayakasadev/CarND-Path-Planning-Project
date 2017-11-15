#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

// Waypoint map to read from
const std::string map_file_ = "../data/highway_map.csv";

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

// fixed speed limit
const double speed_limit = 50.0;

const short spacing = 30; // spacing in m between s way points

const float speed_adjust_rate = .224;

const float time_interval = .02;

const float mph_2_mps = 2.23694;

#endif //CONSTANTS_H
