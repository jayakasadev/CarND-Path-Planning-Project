//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
// #include "../Eigen-3.3/Eigen/Core"
// #include "../Eigen-3.3/Eigen/QR"
#include "../Eigen-3.3/Eigen/Dense"
#include "../utilities/utilities.h"
#include "../utilities/map.h"
#include "../utilities/constants.h"
#include "../utilities/car_state.h"
#include "../utilities/vehicle.h"
#include "../utilities/json.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class trajectory {
private:
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    MatrixXd A;
    VectorXd d;
    VectorXd s;

    void calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, bool s_or_d);

public:
    trajectory(){

        A = MatrixXd(3, 3);
        s = VectorXd(6);
        d = VectorXd(6);

        // cout << "trajectory constructor"  << endl;
        A << pow(time_period, 3), pow(time_period, 4), pow(time_period, 5),
                3 * pow(time_period, 2), 4 *pow(time_period, 3), 5 * pow(time_period, 4),
                6 * time_period, 12 * pow(time_period, 2), 20 * pow(time_period, 3);

        // cout << A << endl;

        A = A.inverse();
        // cout << t << endl;
    } // constructor

    ~trajectory(){} // destructor

    void generate(int prev_size, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y, driver &driver,
                  const vector<lane_state> &lane_score, const vector<double> &velocity_score);

    vector<double> getNext_x_vals();

    vector<double> getNext_y_vals();

    double predict(double t, bool s_or_d);
};


#endif //PATH_PLANNING_TRAJECTORY_H
