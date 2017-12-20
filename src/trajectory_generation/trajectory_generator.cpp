//
// Created by jay on 12/5/17.
//

#include "trajectory_generator.h"

double trajectory_generator::calculatePoint(float &t, VectorXd &constants){
    Eigen::VectorXd x(6);
    x << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    return x.transpose() * constants;
}

void trajectory_generator::calculatePoints(VectorXd &constants_S, VectorXd &constants_D){
    x_vals.clear();
    y_vals.clear();

    for(short a = 1;a <= num_points; a++){
        float time = a * refresh_rate;
        double s = calculatePoint(time, constants_S);
        double d = calculatePoint(time, constants_D);

        std::vector<double> xy = mapData->getXY(s, d);
        cout << "s: " << s << " || d: " << d << " time = " << time << endl;
        // cout << "x: " << xy[0] << " || y: " << xy[1] << "\n" << endl;
        x_vals.push_back(xy[0]);
        y_vals.push_back(xy[1]);
    }
}