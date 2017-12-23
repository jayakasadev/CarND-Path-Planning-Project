//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include <math.h>
#include "../enums/drive_mode.h"

// The max s value before wrapping around the track back to 0
const float max_s = 6945.554;

const double mph_to_mps = 0.44704; // 1 mph = 0.44704 m/s

const double max_velocity_mps = 49.99 * mph_to_mps;

const float max_jerk = 50;

const float max_acceleration = 10;

const float spacing = 49.99 * mph_to_mps;

const float num_lanes = 3;

// for sensor_fusion

const short search_field_buffer = 5; // search above and below the buffer for other vehicles

const float search_field_decay = 0.85; // decay the search field size for each lane so I do not search too far

const short search_field_timelimit = 100; // forget the car if I have not seen it in .1 seconds

// trajectory generation

const double refresh_rate = 0.02;

const float time_period = 1.0;

const short num_points = time_period / refresh_rate;

const drive_mode driveMode = REGULAR; // this is the median score

const float velocity_barrier = 0.2; // anything below 10mph is generated differently

const float scalar = .00007114;

#endif //PATH_PLANNING_CONSTANTS_H
