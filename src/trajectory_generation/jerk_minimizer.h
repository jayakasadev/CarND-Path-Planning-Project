#ifndef PATH_PLANNING_JERK_MINIMIZER_H
#define PATH_PLANNING_JERK_MINIMIZER_H

#include <algorithm>
#include "../Eigen-3.3/Eigen/Dense"
#include "../utilities/utilities.h"
#include "../utilities/car_state.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class jerk_minimizer {
private:
    VectorXd d;
    VectorXd s;

    bool viable;

public:

    jerk_minimizer(){
        s = VectorXd(6);
        d = VectorXd(6);

        viable = true;
    }

    ~jerk_minimizer(){}

    void calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, double time, bool s_or_d);

    double predict(double t, bool s_or_d);

    inline double getCost(short curr_lane, short target_lane, double curr_vel, scores &score){
        // cout << "COST FUNCTION: " << endl;
        // cout << "curr_lane: " << curr_lane << " || target_lane: " << target_lane << " || curr_vel: " << curr_vel << endl;
        // assuming already viable
        if(score.getLaneScore(target_lane) == OBSTRUCTION){
            return 1; // no point going further
        }
        double d_lane= (abs(target_lane - curr_lane) / double(total_lanes));
        double available_space = ((2 * spacing - (score.getDistanceFrontScore(target_lane) - score.getDistanceBackScore(target_lane))) / (2 * spacing));
        double d_s = ((spacing - score.getDistanceFrontScore(target_lane)) / spacing);
        double d_v = ((speed_limit - (score.getVelocityScore(target_lane) - curr_vel)) / (2 * speed_limit));

        // cout << "d_lane: " << d_lane << " || available_space: " << available_space << " || d_s: " << d_s << " || d_v: " << d_v << endl;

        if(curr_lane == target_lane){
            // cout << "cost = " << ((d_lane + d_s + d_v) / 3.0) << endl;
            return ((d_lane + d_s + d_v) / 3.0);
        }
        // cout << "cost = " << ((d_lane + d_s + d_v + available_space) / 4.0) << endl;
        return ((d_lane + d_s + d_v + available_space) / 4.0);
    }

    inline bool getViability(){
        /*
        if(viable){
            cout << "Viable" << endl;
        } else {
            cout << "NOT Viable" << endl;
        }
         */

        return viable;
    }

    inline void resetViability(){
        viable = true;
    }
};


#endif //PATH_PLANNING_JERK_MINIMIZER_H
