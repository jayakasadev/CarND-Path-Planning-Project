#ifndef PATH_PLANNING_ROAD_CONSTANTS_H
#define PATH_PLANNING_ROAD_CONSTANTS_H

/**
 * Values are fixed in stone according to the rules of the simulator
 */

// The max s value before wrapping around the track back to 0
const float max_s = 6945.554;

const float num_lanes = 3;

const float max_jerk = 50; // m / s / s / s

const float max_acceleration = 10; // m / s / s

const double max_velocity = 50.0;

const double mph_to_mps = 0.44704; // 1 mph = 0.44704 m/s

const double max_velocity_mps = max_velocity * mph_to_mps;

const float time_period = 1.0;

// application call is made ever 200 ms on average
const double refresh_rate = 0.02;

const short num_points = time_period / refresh_rate;

#endif //PATH_PLANNING_ROAD_CONSTANTS_H
