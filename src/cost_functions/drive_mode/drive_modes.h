#ifndef PATH_PLANNING_DRIVE_MODES_H
#define PATH_PLANNING_DRIVE_MODES_H

#include "../../constants/simulator_constants.h"
#include "../../tunable_params/cost_function_tunable.h"

class mode{
protected:
    float buffer_distance;
    float desired_acceleration;
    float desired_jerk;
public:
    inline float getBufferDistance(){
        return buffer_distance;
    }

    inline float getDesiredAcceleration(){
        return desired_acceleration;
    }

    inline float getDesiredJerk(){
        return desired_jerk;
    }
};

class regular_mode : public mode{
public:
    regular_mode(){
        buffer_distance = buffer_interval * regular_buffer_multiple; // 5m
        desired_acceleration = max_acceleration * regular_rate; // 3.33 m/s/s
        desired_jerk = max_jerk * regular_rate; // 16.67 m/s/s/s
    }

    ~regular_mode(){}
};

class sport_mode : public mode{
public:
    sport_mode(){
        buffer_distance = buffer_interval * sport_buffer_multiple; // 2.5m
        desired_acceleration = max_acceleration * sport_rate; // 5 m/s/s
        desired_jerk = max_jerk * sport_rate; // 25 m/s/s/s
    }

    ~sport_mode(){}
};

class eco_mode : public mode{
public:
    eco_mode(){
        buffer_distance = buffer_interval * eco_buffer_multiple; // 7.5m
        desired_acceleration = max_acceleration * eco_rate; // 2 m/s/s
        desired_jerk = max_jerk * eco_rate; // 10 m/s/s/s
    }

    ~eco_mode(){}
};

#endif //PATH_PLANNING_DRIVE_MODES_H
