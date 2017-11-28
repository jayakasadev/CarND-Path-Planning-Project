#include "trajectory.h"

void trajectory::generate(int prev_size,nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y, driver &driver,
                          const vector<lane_state> &lane_score, const vector<double> &velocity_score){

}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}

void trajectory::calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, bool s_or_d){
    VectorXd b(3);
    VectorXd c(3);

    b << (xf - (x + x_dot * time_period + 0.5 * x_dot_dot * pow(time_period, 2))),
            (xf_dot - (x_dot + x_dot_dot * time_period)),
            (xf_dot_dot - x_dot_dot);

    c = A * b;
    if(s_or_d) // this is s
        s << x, x_dot, x_dot_dot, c[0], c[1], c[2];
    else // this is d
        d << x, x_dot, x_dot_dot, c[0], c[1], c[2];
}

double trajectory::predict(double t, bool s_or_d) {
    VectorXd x(6);
    x << 1.0, time_period, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    if(s_or_d) // this is s
        return x.transpose() * s;
    return x.transpose() * d; // else d
}