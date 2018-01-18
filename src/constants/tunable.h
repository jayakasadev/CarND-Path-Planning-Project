//
// Created by jay on 1/8/18.
//

#ifndef PATH_PLANNING_DRIVE_SETTINGS_H
#define PATH_PLANNING_DRIVE_SETTINGS_H

#include "../enums/drive_mode.h"

const drive_mode driveMode = drive_mode ::REGULAR; // desired drive mode

const double max_velocity_desired = 49.99; // desired max speed

const float s_search_start_point = 8.0f; // start searching from 8m in front of current position

const short num_generated_s_points = 10; // randomly generate 10 points

const float velocity_barrier = 0.2; // anything below 10mph is generated differently

#endif //PATH_PLANNING_DRIVE_SETTINGS_H
