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
    short first;

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

    inline double calculateRoot(double jerk, double acceleration, double delta_v){
        double left = - acceleration;
        double right = sqrt(pow(acceleration, 2) - 4 * jerk * delta_v);
        double bottom = 2 * jerk;

        double plus = (left - right) / bottom;
        double minus = (left + right) / bottom;
        if(plus < 0) return minus;
        else if(minus < 0) return plus;
        return min(plus, minus);
    }

    inline double calculateTimeS(double &x_dot, double &x_dot_dot, double &xf_dot, double &time){
        // cout << "calculateTimeS: [ x_dot = " << x_dot << " || x_dot_dot = " << x_dot_dot << " || xf_dot = " << xf_dot << " || time = " << time << " ]" << endl;
        if(x_dot_dot >= max_acceleration && xf_dot > x_dot){ // accellerate
            x_dot_dot = max_acceleration - 0.01;
            time = (xf_dot - x_dot) / x_dot_dot;
        }
        else if(x_dot_dot <= -max_acceleration && xf_dot < x_dot){ // decelerate
            x_dot_dot = -max_acceleration + 0.01;
            time = (xf_dot - x_dot) / x_dot_dot;
        }
        // cout << "x_dot_dot = " << x_dot_dot  << " || time = " << time << endl;
    }

    inline double calculateTimeD(double &x, double &xf, double &x_dot, double &x_dot_dot, double &xf_dot, double &time){
        // cout << "calculateTimeD: [ x_dot = " << x_dot << " || x_dot_dot = " << x_dot_dot << " || xf_dot = " << xf_dot << " || time = " << time << " || x = " << x << " || xf = " << xf << " ]" << endl;
        if(x_dot_dot >= max_acceleration && xf > x){ // accelerate right
            x_dot_dot = max_acceleration - 0.01;
            time = (xf_dot - x_dot) / x_dot_dot;
        }
        else if(x_dot_dot <= -max_acceleration && xf < x){ // accelerate left
            x_dot_dot = -max_acceleration + 0.01;
            time = (xf_dot - x_dot) / x_dot_dot;
        }
        // cout << "x_dot_dot = " << x_dot_dot  << " || time = " << time << endl;
    }

public:
    trajectory(){
        for(short a = 0; a < total_lanes; a++){
            minimizers.push_back(jerk_minimizer());
            costs.push_back(1.0);
        }

        first = 0;
    } // constructor

    ~trajectory(){} // destructor

    void generate(driver &driver, scores &score, map_data &mapData);

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();



};


#endif //PATH_PLANNING_TRAJECTORY_H
