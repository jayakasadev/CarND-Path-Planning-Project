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
#include "../constants/constants.h"
#include "../map/map.h"
#include "../trajectory_option/trajectory_option.h"
#include "../vehicle/driver.h"

using namespace Eigen;
using namespace std;

class trajectory_generator {
private:
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    const map_data * mapData;
    double sf;
    double df;
    double sf_dot;
    double df_dot;
    double sf_dot_dot;
    double df_dot_dot;
    double ps;
    double pd;
    double px;
    double py;
    double diff;
    VectorXd *t;
    ofstream outputfile;

    const string filename = "../data/points.txt";

    double calculatePoint(double &t, VectorXd &constants);

public:
    trajectory_generator(const map_data &mapData){
        this->mapData = &mapData;
        t = new VectorXd(6);

        sf = 0;
        sf_dot = 0;
        sf_dot_dot = 0;

        df = 0;
        df_dot = 0;
        df_dot_dot = 0;
        outputfile.open(filename);

        ps = 0;
        pd = 0;
        px = 0;
        py = 0;
         diff = 0;
    }
    ~trajectory_generator(){
        outputfile.close();
    }

    void calculatePoints(trajectory_option &s_option, trajectory_option &d_option, short size);

    inline std::vector<double> getXVals(){
        return x_vals;
    }

    inline std::vector<double> getYVals(){
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
