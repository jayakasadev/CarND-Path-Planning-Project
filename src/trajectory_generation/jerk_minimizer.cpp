//
// Created by jay on 11/28/17.
//

#include "jerk_minimizer.h"

void jerk_minimizer::calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, double time, bool s_or_d){

    if(s_or_d)
        cout << "\tjerk_minimizer::calculate S: " << endl;
    else
        cout << "\tjerk_minimizer::calculate D: " << endl;

    // cout << "\t\tx: " << x << " || x_dot: " << x_dot << " || x_dot_dot: " << x_dot_dot << endl;
    // cout << "\t\txf: " << xf << " || xf_dot: " << xf_dot << " || xf_dot_dot: " << xf_dot_dot << endl;
    // cout << "time: " << time << endl;


    MatrixXd A(3, 3);

    // cout << "trajectory constructor"  << endl;
    A << pow(time, 3), pow(time, 4), pow(time, 5),
            3 * pow(time, 2), 4 *pow(time, 3), 5 * pow(time, 4),
            6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

    // cout << "temp: " << temp << endl;

    A = A.inverse();
    // cout << "A^-1: " << A << endl;

    VectorXd b(3);

    b << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
            (xf_dot - (x_dot + x_dot_dot * time)),
            (xf_dot_dot - x_dot_dot);

    // cout << "b: " << b.transpose() << endl;

    VectorXd c = A * b;
    // cout << "c: " << c << endl;
    if(s_or_d){
        // this is s
        s << x, x_dot, x_dot_dot, c[0], c[1], c[2];
        cout << "\t\ts: "<< s.transpose() << endl;
        // viability check
        if(abs(s[2]) >= max_acceleration) viable = false; // check that acceleration is below  +/- 10m/s^2
        else if(abs(s[3]) >= max_jerk) viable = false; // check that jerk is below +/- 50m/s^2
    } else {
        // this is d
        d << x, x_dot, x_dot_dot, c[0], c[1], c[2];
        cout << "\t\td: "<< d.transpose() << endl;
        // viability check
        if(abs(d[2]) >= max_acceleration) viable = false; // check that acceleration is below  +/- 10m/s^2
        else if(abs(d[3]) >= max_jerk) viable = false; // check that jerk is below +/- 50m/s^2
    }
}

double jerk_minimizer::predict(double t, bool s_or_d) {
    // cout << "PREDICT: ";
    VectorXd x(6);
    x << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    double out = 0;
    if(s_or_d){
        // this is s
        out = x.transpose() * s;
        // cout << "s: "<< out << endl;
    } else {
        out = x.transpose() * d; // else d
        // cout << "d: "<< out << endl;
    }
    return out;
}