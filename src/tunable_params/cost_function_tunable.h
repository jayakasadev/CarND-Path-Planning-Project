#ifndef PATH_PLANNING_COST_FUNCTION_TUNABLE_H
#define PATH_PLANNING_COST_FUNCTION_TUNABLE_H

const float lane_edge_buffer = 1;

enum drive_mode{
    SPORT, REGULAR, ECONOMY
};

// play with these to change drive mode behavior

const float buffer_interval = 2.5;

const drive_mode driveMode = drive_mode::REGULAR; // desired drive mode

const float regular_rate = 1/3;

const short regular_buffer_multiple = 2;

// const drive_mode driveMode = drive_mode::SPORT; // desired drive mode

const float sport_rate = 1/2;

const short sport_buffer_multiple = 1;


// const drive_mode driveMode = drive_mode::ECONOMY; // desired drive mode

const float eco_rate = 1/5;

const short eco_buffer_multiple = 3;

#endif //PATH_PLANNING_COST_FUNCTION_TUNABLE_H
