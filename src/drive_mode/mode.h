//
// Created by jay on 1/12/18.
//

#ifndef PATH_PLANNING_MODE_H
#define PATH_PLANNING_MODE_H

#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/road_constants.h"
#include "../constants/sensor_fusion_constants.h"

class regular_mode {
protected:
    inline double logistic(double x){
        return 2.0 / (1 + exp(-x)) - 1.0;
    }
    float buffer_distance;
    float desired_acceleration;
    float desired_jerk;

public:

    regular_mode(){
        buffer_distance = buffer_interval * 2; // 5m
        desired_acceleration = max_acceleration * (1 / 3); // 3.33 m/s/s
        desired_jerk = max_jerk * (1 / 3); // 16.67 m/s/s/s
    }

    ~regular_mode(){}

    /**
     * Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
     *
     * @param time
     * @return cost
     */
    double time_diff_cost(double time);

    /**
     * Rewards high average speeds.
     *
     * @param s
     * @param time
     * @param targ_s
     * @return cost
     */
    double efficiency_cost(Eigen::VectorXd &s, double time, double targ_s);


    // drive mode dependent behavior

    /**
     * Binary cost function which penalizes collisions.
     *
     * @return true if legal, else false
     */
    bool collision_cost(Eigen::VectorXd &s, Eigen::VectorXd &d, double time);

    /**
     * Calculate log of (average jerk per second / desired jerkper second)
     *
     * @param s
     * @param time
     * @return cost
     */
    double avg_jerk_cost(Eigen::VectorXd &s, double time);

    /**
     * Calculate log of (average acceleration per second/ desired acceleration per second)
     *
     * @param s
     * @param time
     * @return cost
     */
    double avg_acc_cost(Eigen::VectorXd &s, double time);
};

class sport_mode : public regular_mode{
public:
    sport_mode(){
        buffer_distance = buffer_interval; // 2.5m
        desired_acceleration = max_acceleration * (1 / 2); // 5 m/s/s
        desired_jerk = max_jerk * (1 / 2); // 25 m/s/s/s
    }

    ~sport_mode(){}
};

class eco_mode : public regular_mode{
public:
    eco_mode(){
        buffer_distance = buffer_interval * 3; // 7.5m
        desired_acceleration = max_acceleration * (1 / 5); // 2 m/s/s
        desired_jerk = max_jerk * (1 / 5); // 10 m/s/s/s
    }

    ~eco_mode(){}
};

#endif //PATH_PLANNING_MODE_H
