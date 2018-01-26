//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_CITY_PLANNER_H
#define PATH_PLANNING_CITY_PLANNER_H


#include "planner.h"

class city_planner : public planner{// behavior based on velocity
private:

public:
    city_planner() : planner(){
        std::cout << "city_planner constructor" << std::endl;
    }

    ~city_planner(){
        std::cout << "city_planner destructor" << std::endl;
    }

    city_planner(const city_planner &city_planner){
        std::cout << "city_planner copy constructor" << std::endl;
    }

    void calculate();
};


#endif //PATH_PLANNING_CITY_PLANNER_H
