#include "trajectory_option.h"

void trajectory_option::calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, Eigen::VectorXd &vector){
    Eigen::VectorXd xvals = Eigen::VectorXd::Zero(3);

    xvals << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
            (xf_dot - (x_dot + x_dot_dot * time)),
            (xf_dot_dot - x_dot_dot);

    Eigen::VectorXd constants = *t * xvals;
    vector << x, x_dot, x_dot_dot, constants[0], constants[1], constants[2];
}



void trajectory_option::print(){
    std::cout << "trajectory_option\ttime: " << time <<  std::endl;
}