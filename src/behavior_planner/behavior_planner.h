//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <future>
#include <vector>

#include "../tunable_params/behavior_planning_tunable.h"
#include "../vehicle/driver.h"
#include "../trajectory/trajectory.h"
#include "planning/planner.h"

using namespace Eigen;

class behavior_planner {
private:
    std::unique_ptr<pointer_pool<planner>> cityPlanners;
    std::unique_ptr<pointer_pool<planner>> highwayPlanners;
    std::shared_ptr<driver> car;

public:

    behavior_planner(std::shared_ptr<driver> car, std::unique_ptr<pointer_pool<planner>> &cityPlanner,
                     std::unique_ptr<pointer_pool<planner>> &highwayPlanner){
        std::cout << "behavior_planner constructor" << std::endl;
        this->car = car;
        this->cityPlanners = std::move(cityPlanners);
        this->highwayPlanners = std::move(highwayPlanners);

    }

    ~behavior_planner(){
        std::cout << "behavior_planner destructor" << std::endl;
    }

    std::vector<trajectory> plan();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
