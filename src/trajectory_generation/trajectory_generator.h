//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include <vector>
#include <iostream>
#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/constants.h"
#include "../map/map.h"
#include "../trajectory_option/trajectory_option.h"

using namespace Eigen;
using namespace std;

class trajectory_generator {
private:
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    const map_data * mapData;
    short num_points_to_gen;

    double calculatePoint(float &t, VectorXd &constants);

public:
    trajectory_generator(const map_data &mapData){
        this->mapData = &mapData;
        num_points_to_gen = num_points;
    }
    ~trajectory_generator(){}

    void calculatePoints(trajectory_option &s_option, trajectory_option &d_option);

    inline std::vector<double> getXVals(){
        return x_vals;
    }

    inline std::vector<double> getYVals(){
        return y_vals;
    }

    inline void adjustNumPoints(double existing){
        num_points_to_gen = num_points;
        num_points_to_gen -= existing;
    }

};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
