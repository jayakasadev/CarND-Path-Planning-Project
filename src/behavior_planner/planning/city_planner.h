//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_CITY_PLANNER_H
#define PATH_PLANNING_CITY_PLANNER_H


#include "planner.h"

class city_planner : public planner{// behavior based on velocity
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
