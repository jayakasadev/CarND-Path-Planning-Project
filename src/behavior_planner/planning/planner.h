#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "../../vehicle/driver.h"
#include "../../utilities/pools/pointer_pool.h"
#include "../trajectory_calculator/trajectory_calculator.h"
#include "../../trajectory/trajectory.h"
#include "../../vehicle/traffic.h"
#include "../points/points.h"

class planner{
protected:
    std::vector<double> generated;
    std::shared_ptr<pointer_pool<points>> calculation_s;
    std::shared_ptr<pointer_pool<points>> calculation_d;
    std::shared_ptr<driver> car;
    short lane;

    /**
     * Method to generate random points
     */
    inline void generateRandomPoints(){
        generated.clear();
        for(short a = 0; a < num_generated_s_points; a++){
            double random = (double)rand() / RAND_MAX;
            generated.push_back(s_search_start_point + random * (spacing - s_search_start_point));
        }
    }

    /**
     * Method to print randomly generated points
     */
    inline void printGenerated(){
        for(short a = 0; a < generated.size(); a++){
            std::cout << "generated: " << generated[a] << std::endl;
        }
    }
public:

    planner(){
        // std::cout << "planner constructor" << std::endl;
    }

    ~planner(){
        std::cout << "planner destructor" << std::endl;
    }

    planner(const planner &planner){
        // std::cout << "planner copy constructor" << std::endl;
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

    void setLane(short lane){
        this->lane = lane;
    }

    void setCar(std::shared_ptr<driver> car){
        this->car = car;
    }

    void setCalculations(std::shared_ptr<pointer_pool<points>> calculation_s, std::shared_ptr<pointer_pool<points>> calculation_d){
        this->calculation_s = calculation_s;
        this->calculation_d = calculation_d;
    }

    friend std::ostream& operator <<(std::ostream& os, planner& planner){
        os << "planner\tlane:" << planner.lane;
        return os;
    }
};

#endif //PATH_PLANNING_PLANNER_H
