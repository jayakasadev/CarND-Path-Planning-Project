#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_FACTORY_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_FACTORY_H

// static factory that builds the behavior_planner as needed

// attaches an object pool of calculators

// attaches planners

// attaches shared_ptrs

#include <iostream>
#include <memory>
#include "behavior_planner.h"
#include "../utilities/unique_ptr_helper.h"
#include "../utilities/pools/synch_pool.h"
#include "../cost_functions/cost_function.h"
#include "planning/city_planner.h"
#include "planning/highway_planner.h"

class behavior_planner_factory{
private:
    std::unique_ptr<pointer_pool<planner>> cityPlanners;
    std::unique_ptr<pointer_pool<planner>> highwayPlanners;
    std::shared_ptr<pointer_pool<trajectory_calculator>> calculators;

    void build(std::shared_ptr<driver> car, std::shared_ptr<cost_function> costFunction);

public:
    behavior_planner_factory(){
        // std::cout << "behavior_planner_factory constructor" << std::endl;
    }
    ~behavior_planner_factory(){
        // std::cout << "behavior_planner_factory destructor" << std::endl;
        assert(!cityPlanners); // making sure that the factory no longer owns the planner pools
        assert(!highwayPlanners); // making sure that the factory no longer owns the planner pools
        assert(calculators.use_count() > 0); // make sure the calculators are shared
    }

    std::unique_ptr<behavior_planner> getInstance(std::shared_ptr<driver> car,
                                                  std::shared_ptr<cost_function> costFunction,
                                                  std::shared_ptr<std::vector<trajectory>> path);
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_FACTORY_H
