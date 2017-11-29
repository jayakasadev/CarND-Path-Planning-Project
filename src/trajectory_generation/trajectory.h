//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "../utilities/utilities.h"
#include "../utilities/map.h"
#include "../utilities/constants.h"
#include "../utilities/car_state.h"
#include "../utilities/vehicle.h"
#include "../utilities/json.hpp"
#include "jerk_minimizer.h"

using namespace std;

class trajectory {
private:
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    jerk_minimizer lane0;
    jerk_minimizer lane1;
    jerk_minimizer lane2;

public:
    trajectory(){} // constructor

    ~trajectory(){} // destructor

    void generate(int prev_size, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y, driver &driver,
                  const vector<lane_state> &lane_score, const vector<double> &velocity_score);

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();


};


#endif //PATH_PLANNING_TRAJECTORY_H
