#ifndef PATH_PLANNING_BEHAVIOR_CONSTANTS_H
#define PATH_PLANNING_BEHAVIOR_CONSTANTS_H

#include "../constants/simulator_constants.h"

// play with these when memory is a constraint

const bool lazy_loading = false;
// behavior planning barrier between city and highway planning

const float velocity_barrier = 0.2; // anything below 10mph is generated differently

const double max_velocity_desired = 49.99; // desired max speed

const double desired_velocity_mps = max_velocity_desired * mph_to_mps;

const double spacing = desired_velocity_mps;

// trajectory searching constants

const float s_search_start_point = 8.0f; // start searching from 8m in front of current position

const short num_generated_s_points = 10; // randomly generate 10 points

const short calculator_instance_limit = 10;

const short trajectory_instance_limit = 10;

const short calculator_instance_max = num_points + 1;

const short trajectory_instance_max = calculator_instance_max * num_generated_s_points;
#endif //PATH_PLANNING_BEHAVIOR_CONSTANTS_H
