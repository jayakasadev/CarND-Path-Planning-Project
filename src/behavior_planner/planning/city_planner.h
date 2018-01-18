//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_CITY_PLANNER_H
#define PATH_PLANNING_CITY_PLANNER_H


#include "planner.h"

class city_planner : public planner{// behavior based on velocity
private:

public:
    city_planner(driver &car, detections &detected, vector<trajectory_option *> &calculators, short lane): planner(car, detected, calculators, lane){
        cout << "city_planner constructor" << endl;
    }

    ~city_planner(){
        cout << "city_planner destructor" << endl;
    }

    void calculate();
};


#endif //PATH_PLANNING_CITY_PLANNER_H
