#include "trajectory_calculator.h"
#include "../../tunable_params/behavior_planning_tunable.h"

void trajectory_calculator::initialize(double time){
    this->time = time;
    t << pow(time, 3), pow(time, 4), pow(time, 5),
            3.0d * pow(time, 2), 4.0d *pow(time, 3), 5.0d * pow(time, 4),
            6.0d * time, 12.0d * pow(time, 2), 20.0d * pow(time, 3);
    // std::cout << *t << std::endl;
    t = t.inverse();
}

void trajectory_calculator::calculate(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, Eigen::VectorXd &vector){
    if(lazy_loading) {
        // only bother rebuilding the matrix if this is a lazy_build object
        initialize(time);
    }

    Eigen::VectorXd xvals = Eigen::VectorXd::Zero(3);

    xvals << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
            (xf_dot - (x_dot + x_dot_dot * time)),
            (xf_dot_dot - x_dot_dot);

    Eigen::VectorXd constants = t * xvals;
    vector << x, x_dot, x_dot_dot, constants[0], constants[1], constants[2];
}