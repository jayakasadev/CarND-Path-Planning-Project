//
// Created by jay on 1/18/18.
//

#ifndef PATH_PLANNING_COST_FUNCTION_H
#define PATH_PLANNING_COST_FUNCTION_H

#include <iostream>
#include "../Eigen-3.3/Eigen/Dense"
#include "../utilities/pools/pointer_pool.h"
#include "../tunable_params/cost_function_tunable.h"
#include "drive_mode/drive_modes.h"
#include "../vehicle/traffic.h"

class cost_function{
private:
    mode currMode;
    std::shared_ptr<pointer_pool<traffic>> detected;

    inline double logistic(double x){
        return 2.0 / (1 + exp(-x)) - 1.0;
    }

    void setDriveMode();

public:

    cost_function(std::shared_ptr<pointer_pool<traffic>> detected){
        // std::cout << "cost_function constructor" << std::endl;
        this->detected = detected;
        setDriveMode();
    }

    ~cost_function(){
        std::cout << "cost_function destructor" << std::endl;
    }
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

#endif //PATH_PLANNING_COST_FUNCTION_H
