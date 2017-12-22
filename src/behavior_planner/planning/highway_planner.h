//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_HIGHWAY_PLANNER_H
#define PATH_PLANNING_HIGHWAY_PLANNER_H

#include "planner.h"

class highway_planner : public planner{// behavior based on velocity
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
