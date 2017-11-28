//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include "utilities.h"
#include "map.h"
#include "constants.h"
#include "car_state.h"
#include "vehicle.h"
#include "json.hpp"

using namespace std;

class trajectory {
private:
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    double ref_vel;

public:
    trajectory():ref_vel(0.0){} // constructor

    ~trajectory(){} // destructor

    void generate(int prev_size, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y, driver &driver,
                  const vector<lane_state> &lane_score, const vector<double> &velocity_score);

    float getCost();

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();
};


#endif //PATH_PLANNING_TRAJECTORY_H
