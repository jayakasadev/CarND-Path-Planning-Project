//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_CITY_PLANNER_H
#define PATH_PLANNING_CITY_PLANNER_H


#include "planner.h"

class city_planner : public planner{// behavior based on velocity
private:

    // weights for the cost function
    // max value is 7500 for a bad score
    // best score is 0

    const double k_j = 1;

    const double k_d = 39.0625;

    const double k_t = 156.25;

    inline double costSD(double time, double diff_s, double diff_d, Eigen::VectorXd &calculated){
        /*
        std::cout << "\tplanner::costD\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, double(time_period)) + k_t * (diff_s) + k_d * pow(diff_d, 2.0d);
    }

public:
    city_planner(driver &car, scores &values, short lane){
        // cout << "city_planner" << endl;
        this->car = &car;
        // car.print();
        this->values = &values;
        this->lane = lane;
    }

    ~city_planner(){}

    void calculate();
};


#endif //PATH_PLANNING_CITY_PLANNER_H
