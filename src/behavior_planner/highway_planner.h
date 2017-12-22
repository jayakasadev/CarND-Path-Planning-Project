//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_HIGHWAY_PLANNER_H
#define PATH_PLANNING_HIGHWAY_PLANNER_H

#include "planner.h"

class highway_planner : public planner{// behavior based on velocity
public:
    highway_planner(driver &car, scores &values){
        this->car = &car;
        this->values = &values;
    }

    ~highway_planner(){}

    void calculateS(short lane, double &score_s, double &time_s, VectorXd &s);

    void calculateD(short lane, double &score_d, double &time_d, VectorXd &d);

};


#endif //PATH_PLANNING_HIGHWAY_PLANNER_H
