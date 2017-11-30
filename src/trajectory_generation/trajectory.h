//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <iostream>
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
    vector<jerk_minimizer> minimizers;
    vector<double> costs;

    inline void resetScores(){
        costs.clear();
        for(short a = 0; a < total_lanes; a++){
            costs.push_back(numeric_limits<double>::max());
        }
    }

    inline void printScores(){
        for(short a = 0; a < total_lanes; a++){
            cout << "lane: " << a << " || score: " << costs[a] << "\t";
        }
        cout << endl;
    }

    void calculate(driver &driver, scores &score, short &lane);

public:
    trajectory(){
        for(short a = 0; a < total_lanes; a++){
            minimizers.push_back(jerk_minimizer());
            costs.push_back(numeric_limits<double>::max());
        }
    } // constructor

    ~trajectory(){} // destructor

    void generate(int prev_size, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y,
                  driver &driver, scores &score);

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();



};


#endif //PATH_PLANNING_TRAJECTORY_H
