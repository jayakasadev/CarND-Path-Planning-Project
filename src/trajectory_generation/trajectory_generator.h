//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include <vector>
#import "../Eigen-3.3/Eigen/Dense"
#include "../constants/constants.h"
#include "../map/map.h"

using namespace Eigen;

class trajectory_generator {
private:
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    const map_data *mapData;

    double calculatePoint(float t, VectorXd constants);

public:
    trajectory_generator(const map_data &mapData){
        this->mapData = &mapData;
    }
    ~trajectory_generator(){}

    void calculatePoints(VectorXd constants_S, VectorXd constants_D);

    inline std::vector<double> getXVals(){
        return x_vals;
    }

    inline std::vector<double> getYVals(){
        return y_vals;
    }

};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
