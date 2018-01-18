//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_HIGHWAY_PLANNER_H
#define PATH_PLANNING_HIGHWAY_PLANNER_H

#include "planner.h"

class highway_planner : public planner{// behavior based on velocity
private:
    void calculateS();

    void calculateD();

public:
    highway_planner(driver &car, detections &detected, vector<trajectory_option *> &calculators, short lane) : planner(car, detected, calculators, lane){
        cout << "highway_planner constructor" << endl;
    }

    ~highway_planner(){
        cout << "highway_planner destructor" << endl;
    }

    void calculate();

};


#endif //PATH_PLANNING_HIGHWAY_PLANNER_H
