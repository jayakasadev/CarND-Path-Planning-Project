//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <future>
#include <vector>

#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/constants.h"
#include "../enums/vehicle_behavior.h"
#include "../scores/scores.h"
#include "../vehicle/driver.h"
#include "planning/city_planner.h"
#include "planning/highway_planner.h"
#include "../trajectory_option/trajectory_option.h"

using namespace Eigen;
using namespace std;

class behavior_planner {
private:
    vector<highway_planner> highwayPlanner;
    vector<city_planner> cityPlanner;
    scores * values;
    driver * car;

public:
    behavior_planner(driver &car, scores &values){
        this->values = &values;
        this->car = &car;

        for(short a = 0; a < num_lanes; a++){
            highwayPlanner.push_back(highway_planner(car, values, a));
            cityPlanner.push_back(city_planner(car, values, a));
        }
    }

    ~behavior_planner(){}

    vector<trajectory_option> plan();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
