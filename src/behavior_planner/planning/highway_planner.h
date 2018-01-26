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
    highway_planner() : planner(){
        // std::cout << "highway_planner constructor" << std::endl;
    }

    ~highway_planner(){
        std::cout << "highway_planner destructor" << std::endl;
    }

    highway_planner(const highway_planner &planner){
        std::cout << "highway_planner copy constructor" << std::endl;
        this->car = planner.car;
        this->calculation_s = planner.calculation_s;
        this->calculation_d = planner.calculation_d;
        this->lane = planner.lane;

        /*
        std::cout << "lane: " << lane << std::endl;
        std::cout << "calculation_s: " << calculation_s->size() << std::endl;
        std::cout << "calculation_d: " << calculation_d->size() << std::endl;
         */
    }

    void calculate();

};


#endif //PATH_PLANNING_HIGHWAY_PLANNER_H
