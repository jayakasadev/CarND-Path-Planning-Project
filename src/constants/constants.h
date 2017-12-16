//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include <iostream>
#include "../enums/drive_mode.h"

// The max s value before wrapping around the track back to 0
const float max_s = 6945.554;

const float mph_to_mps = 0.44704; // 1 mph = 0.44704 m/s

const float max_velocity_mps = 22.35199;

const float max_jerk = 50;

const float max_acceleration = 10;

const float spacing = 22.35199;

const float num_lanes = 3;

// for sensor_fusion

const short search_field_buffer = 5; // search above and below the buffer for other vehicles

const float search_field_decay = 0.85; // decay the search field size for each lane so I do not search too far

const short search_field_timelimit = 500; // forget the car if I have not seen it in .5 seconds

// trajectory generation

const float refresh_rate = 0.02;

const float barrier_rate = 0.2;

const float velocity_barrier = max_velocity_mps * barrier_rate;

const float time_period = 1.0;

const short num_points = time_period / refresh_rate;

const drive_mode driveMode = REGULAR;

const float k_j = 1 / (max_jerk * max_jerk);

const float k_d = 1 / 6;

const float k_t = .5;

const float k_s = 1 / spacing;


#endif //PATH_PLANNING_CONSTANTS_H
