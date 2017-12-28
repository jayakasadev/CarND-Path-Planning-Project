//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_HIGHWAY_PLANNER_H
#define PATH_PLANNING_HIGHWAY_PLANNER_H

#include "planner.h"

class highway_planner : public planner{// behavior based on velocity
private:

    // weights for the cost function
    // max value is 7500 for a bad score
    // best score is 0

    const double k_j = 1;

    const double k_d = 39.0625;

    const double k_t = 156.25;

    const double k_s = -4;

    const double k_s_bias = 2500;

    const double k_v = 25;

    // cost functions
    inline double costV(double time, double diff, VectorXd &calculated){
        /*
        std::cout << "planner::costS_Vel\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kv = " << k_v << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - double(time_period)) + k_v * pow(diff, 2.0d);
    }

    inline double costS(double time, double diff, VectorXd &calculated){
        /*
        std::cout << "planner::costS\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", ks = " << k_s << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - double(time_period)) + k_s * pow(diff, 2.0d) + k_s_bias;
    }

    inline double costD(double time, double diff, VectorXd &calculated){
        /*
        std::cout << "\tplanner::costD\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - double(time_period)) + k_d * pow(diff, 2.0d);
    }

public:
    highway_planner(driver &car, scores &values, short lane){
        this->car = &car;
        this->values = &values;
        this->lane = lane;
    }

    ~highway_planner(){}

    void calculateS();

    void calculateD();

};


#endif //PATH_PLANNING_HIGHWAY_PLANNER_H
