//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_CITY_PLANNER_H
#define PATH_PLANNING_CITY_PLANNER_H


#include "planner.h"

class city_planner : public planner{// behavior based on velocity
public:

    city_planner(driver &car, scores &values){
        this->car = &car;
        this->values = &values;
    }

    ~city_planner(){}
};


#endif //PATH_PLANNING_CITY_PLANNER_H
