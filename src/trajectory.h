//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "spline.h"
#include "constants.h"
#include "utilities.h"

using namespace std;

class trajectory {
private:
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    // create a spline
    tk::spline spline;
    double ref_vel;

public:
    trajectory():ref_vel(0.0){} // constructor

    ~trajectory(){} // destructor

    void generate(int prev_size, double car_x, double car_y, double car_yaw,
             vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s,
             vector<double> map_waypoints_x, vector<double> map_waypoints_y, double car_s, double speed, int lane);

    float getCost();

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();
};


#endif //PATH_PLANNING_TRAJECTORY_H
