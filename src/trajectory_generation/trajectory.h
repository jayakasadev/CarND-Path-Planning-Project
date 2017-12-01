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
#include "jerk_minimizer.h"

using namespace std;

class trajectory {
private:
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<jerk_minimizer> minimizers;
    vector<double> costs;

    inline void reset(){
        costs.clear();
        for(short a = 0; a < total_lanes; a++){
            costs.push_back(1);
        }

        next_y_vals.clear();
        next_x_vals.clear();
    }

    inline void printScores(){
        cout << "SCORES: " << endl;
        for(short a = 0; a < total_lanes; a++){
            cout << "\tlane: " << a << " || score: " << costs[a] << endl;
        }
    }

    inline short minScoreIndex(){
        short index = 0;
        for(short a = 1; a < total_lanes; a++){
            if(costs[index] > costs[a]){
                index = a;
            }
        }
        return index;
    }

    void calculate(driver &driver, scores &score, short &lane);

public:
    trajectory(){
        for(short a = 0; a < total_lanes; a++){
            minimizers.push_back(jerk_minimizer());
            costs.push_back(1.0);
        }
    } // constructor

    ~trajectory(){} // destructor

    void generate(driver &driver, scores &score, map_data &mapData);

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();



};


#endif //PATH_PLANNING_TRAJECTORY_H
