#include "behavior_planner_factory.h"
#include "planning/city_planner.h"
#include "planning/highway_planner.h"


std::unique_ptr<behavior_planner> behavior_planner_factory::getInstance(std::shared_ptr<driver> car, std::shared_ptr<cost_function> costFunction){
    // std::cout << "behavior_planner_factory::getInstance" << std::endl;
    build(car, costFunction);
    // std::cout << "build complete" << std::endl;
    if(cityPlanners->isEmpty() || highwayPlanners->isEmpty()){
        throw std::logic_error("behavior planner was not properly built");
    }
    std::unique_ptr<behavior_planner> instance = make_unique<behavior_planner>(car, std::move(cityPlanners), std::move(highwayPlanners));
    return instance;
}

void behavior_planner_factory::build(std::shared_ptr<driver> car, std::shared_ptr<cost_function> costFunction){
    // std::cout << "behavior_planner_factory::build" << std::endl;

    if(lazy_loading){
        calculators = std::make_shared<synch_unique_pool<trajectory_calculator>>();
        for(short a = 0; a < calculator_instance_limit; a++){
            calculators->release(new trajectory_calculator());
            // TODO implement
        }

    } else{
        calculators = std::make_shared<shared_pool<trajectory_calculator>>();
        for(short a = 0; a < calculator_instance_max; a++){
            // std::cout << "adding new obj" << std::endl;
            trajectory_calculator * ptr = new trajectory_calculator();
            ptr->initialize(time_period + a * refresh_rate);
            calculators->add(ptr);
            // std::cout << *ptr << std::endl;
        }
    }

    cityPlanners = make_unique<shared_pool<planner>>();
    highwayPlanners = make_unique<shared_pool<planner>>();

    for(short a = 0; a < num_lanes; a++){

        city_planner * city = new city_planner();
        highway_planner * highway = new highway_planner();

        std::shared_ptr<pointer_pool<points>> calculation_s = std::make_shared<shared_pool<points>>();
        std::shared_ptr<pointer_pool<points>> calculation_d = std::make_shared<shared_pool<points>>();
        for(short b = 0; b < num_generated_s_points; b++){
            // build trajectories for d calculations
            std::shared_ptr<pointer_pool<trajectory>> calculated_points;
            if(lazy_loading){
                calculated_points = std::make_shared<synch_unique_pool<trajectory>>();
                // TODO implement
            } else {
                calculated_points = std::make_shared<shared_pool<trajectory>>();
                for(short c = 0; c <= num_points; c++){
                    trajectory * ptr = new trajectory();
                    ptr->setTime(time_period + c * refresh_rate);
                    ptr->setType(S);
                    calculated_points->add(ptr);
                    // std::cout << *ptr << std::endl;
                }
            }

            // build point for s point
            points * ptr = new points(calculators, calculated_points);
            calculation_s->add(ptr);
            // std::cout << *ptr << std::endl;
        }

        // build trajectories for d calculations
        std::shared_ptr<pointer_pool<trajectory>> calculated_points;
        if(lazy_loading){
            calculated_points = std::make_shared<synch_unique_pool<trajectory>>();
            // TODO implement
        } else {
            calculated_points = std::make_shared<shared_pool<trajectory>>();
            for(short b = 0; b <= num_points; b++){
                trajectory * ptr = new trajectory();
                ptr->setTime(time_period + b * refresh_rate);
                ptr->setType(D);
                calculated_points->add(ptr);
                // std::cout << *ptr << std::endl;
            }
        }

        // build point for d point
        points * ptr = new points(calculators, calculated_points);
        calculation_d->add(ptr);
        // std::cout << *ptr << std::endl;

        // build the city and highway planners with the shared data sources
        city->setCalculations(calculation_s, calculation_d);
        city->setCar(car);
        city->setLane(a);

        highway->setCalculations(calculation_s, calculation_d);
        highway->setCar(car);
        highway->setLane(a);

        cityPlanners->add(city);
        highwayPlanners->add(highway);
    }
}