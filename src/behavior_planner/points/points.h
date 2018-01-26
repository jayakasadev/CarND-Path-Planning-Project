//
// Created by jay on 1/25/18.
//

#ifndef PATH_PLANNING_POINTS_H
#define PATH_PLANNING_POINTS_H

#include <iostream>
#include "../../utilities/pools/pointer_pool.h"
#include "../trajectory_calculator/trajectory_calculator.h"
#include "../../trajectory/trajectory.h"

class points {
private:
    std::shared_ptr<pointer_pool<trajectory_calculator>> calculators;
    std::shared_ptr<pointer_pool<trajectory>> calculated_points;
public:
    points(std::shared_ptr<pointer_pool<trajectory_calculator>> calculators, std::shared_ptr<pointer_pool<trajectory>> calculated_points){
        // std::cout << "points constructor" << std::endl;
        this->calculators = calculators;
        this->calculated_points = calculated_points;
    }

    ~points(){
        std::cout << "points destructor" << std::endl;
    }

    points(const points &points){
        // std::cout << "points copy constructor" << std::endl;
        this->calculators = points.calculators;
        this->calculated_points = points.calculated_points;
    }

    void calculate(double s, double d);

    std::shared_ptr<trajectory> findBest();

    friend std::ostream& operator <<(std::ostream& os, points& obj){
        os << "points\tsize:" << obj.calculated_points->size();
        return os;
    }
};


#endif //PATH_PLANNING_POINTS_H
