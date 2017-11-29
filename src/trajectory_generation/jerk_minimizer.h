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
    double s_t;
    double d_t;
    bool viable;

    inline double calculateTime(double x, double x_dot, double x_dot_dot, double xf){
        double minus = (-1 * x_dot - sqrt(pow(x_dot, 2) - 4 * x_dot_dot * (x - xf))) / (2 * x_dot_dot);
        double plus = (-1 * x_dot + sqrt(pow(x_dot, 2) - 4 * x_dot_dot * (x - xf))) / (2 * x_dot_dot);
        return max(minus, plus);
    }

    inline bool viabilityCheck();

public:

    jerk_minimizer(){
        s = VectorXd(6);
        d = VectorXd(6);
    }

    ~jerk_minimizer(){}

    void calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, bool s_or_d);

    double predict(double t, bool s_or_d);

    double cost(short curr_lane, short target_lane, double curr_vel, double target_vel, double curr_s, double target_s, lane_state state);

    inline double getS_Time(){
        return s_t;
    }

    inline double getD_Time(){
        return d_t;
    }

    inline bool getViability(){
        return viable;
    }
};


#endif //PATH_PLANNING_JERK_MINIMIZER_H
