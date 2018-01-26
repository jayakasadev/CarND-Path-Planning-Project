//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include <vector>
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>

#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/simulator_constants.h"
#include "map/map.h"
#include "../trajectory/trajectory.h"

using namespace Eigen;
using namespace std;

class trajectory_generator {
private:
    std::shared_ptr<std::vector<double>> x_vals;
    std::shared_ptr<std::vector<double>> y_vals;

    std::unique_ptr<map_data> mapData;
    std::shared_ptr<vector<trajectory>> path;

    double sf;
    double df;

    double sf_dot;
    double df_dot;

    double sf_dot_dot;
    double df_dot_dot;

    ofstream outputfile;
    const string filename = "../data/points.txt";

public:
    trajectory_generator(std::shared_ptr<vector<trajectory>> path){
        sf = 0;
        sf_dot = 0;
        sf_dot_dot = 0;

        df = 0;
        df_dot = 0;
        df_dot_dot = 0;

        outputfile.open(filename);

        this->mapData = make_unique<map_data>();
        this->path = path;
    }
    ~trajectory_generator(){
        outputfile.close();
    }

    void calculatePoints(short size);

    inline std::shared_ptr<std::vector<double>> getXVals(){
        return x_vals;
    }

    inline std::shared_ptr<std::vector<double>> getYVals(){
        return y_vals;
    }

    inline vector<double> sfVals(){
        return {sf, sf_dot, sf_dot_dot};
    }

    inline vector<double> dfVals(){
        return {df, df_dot, df_dot_dot};
    }
};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
