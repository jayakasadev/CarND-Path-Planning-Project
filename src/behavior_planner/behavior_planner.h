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
#include "city_planner.h"
#include "highway_planner.h"
#include "../trajectory_option/trajectory_option.h"

using namespace Eigen;
using namespace std;

class behavior_planner {
private:
    highway_planner *highwayPlanner;
    city_planner *cityPlanner;
    scores *values;
    trajectory_option * option;
    driver * car;

public:
    behavior_planner(driver &car, scores &values, trajectory_option &option){
        this->values = &values;
        this->option = &option;
        this->car = &car;

        this->highwayPlanner = new highway_planner(car, values);
        this->cityPlanner = new city_planner(car, values);
    }

    ~behavior_planner(){
        delete highwayPlanner;
        delete cityPlanner;
    }

    void bestOption();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
