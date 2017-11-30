//
// Created by jay on 11/28/17.
//

#include "jerk_minimizer.h"

void jerk_minimizer::calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, bool s_or_d){
    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd c(3);

    double t = calculateTime(x, x_dot, x_dot_dot, xf);
    if(s_or_d) s_t = t;
    else d_t = t;

    // cout << "trajectory constructor"  << endl;
    A << pow(t, 3), pow(t, 4), pow(t, 5),
            3 * pow(t, 2), 4 *pow(t, 3), 5 * pow(t, 4),
            6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

    // cout << A << endl;

    A = A.inverse();
    // cout << t << endl;

    b << (xf - (x + x_dot * t + 0.5 * x_dot_dot * pow(t, 2))),
            (xf_dot - (x_dot + x_dot_dot * t)),
            (xf_dot_dot - x_dot_dot);

    c = A * b;
    if(s_or_d) // this is s
        s << x, x_dot, x_dot_dot, c[0], c[1], c[2];
    else // this is d
        d << x, x_dot, x_dot_dot, c[0], c[1], c[2];

    // viability check
    if(s[2] >= max_acceleration) viable = false; // check that acceleration is below 10m/s^2
    else if(d[3] >= max_jerk) viable = false; // check that jerk is below 50m/s^2
    else viable = true;
}

double jerk_minimizer::predict(double t, bool s_or_d) {
    VectorXd x(6);
    x << 1.0, time_period, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    if(s_or_d) // this is s
        return x.transpose() * s;
    return x.transpose() * d; // else d
}

double jerk_minimizer::getCost(short curr_lane, short target_lane, double curr_vel, scores &score){
    // assuming already viable
    double slope = -1;
    if(score.getLaneScore(target_lane) == OPEN){
        slope += abs(target_lane - curr_lane)/100; // changing lanes means slope goes down and cost goes up for similar actions
    } else if(score.getLaneScore(target_lane) == OBSTRUCTION){
        return numeric_limits<double>::max(); // no point going further
    } else {
        // follow
    }
    return 0;
}