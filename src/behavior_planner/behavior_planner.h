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

using namespace Eigen;
using namespace std;

class trajectory_option{
public:
    float score;
    VectorXd s;
    VectorXd d;

    trajectory_option(){
        score = 0;
    }
    ~trajectory_option(){}
};

class behavior_planner {
private:
    driver *car;
    scores *values;

    trajectory_option calculate(short lane);

    trajectory_option calculateSport(short lane);

    trajectory_option calculateEconomy(short lane);

public:
    behavior_planner(driver &car, scores &values){
        this->car = &car;
        this->values = &values;
    }

    ~behavior_planner(){}

    vector<VectorXd> bestOption();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
