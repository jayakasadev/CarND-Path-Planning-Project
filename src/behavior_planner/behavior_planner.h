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

using namespace Eigen;
using namespace std;

struct trajectory_option{
    double score_s;
    double score_d;
    VectorXd s;
    VectorXd d;

    trajectory_option(){
        this->score_s = numeric_limits<float>::max();
        this->score_d = numeric_limits<float>::max();
        this->s = VectorXd::Zero(6);
        this->d = VectorXd::Zero(6);
    }
    ~trajectory_option(){}
};

class behavior_planner {
private:
    highway_planner *highwayPlanner;
    city_planner *cityPlanner;
    scores *values;

public:
    behavior_planner(driver &car, scores &values){
        this->values = &values;
        this->highwayPlanner = new highway_planner(car, values);
        this->cityPlanner = new city_planner(car, values);
    }

    ~behavior_planner(){
        delete highwayPlanner;
        delete cityPlanner;
    }

    vector<VectorXd> bestOption();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
