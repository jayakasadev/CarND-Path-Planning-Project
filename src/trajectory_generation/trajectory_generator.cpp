//
// Created by jay on 12/5/17.
//

#include "trajectory_generator.h"

double trajectory_generator::calculatePoint(float &t, VectorXd &constants){
    Eigen::VectorXd x(6);
    x << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    return x.transpose() * constants;
}

void trajectory_generator::calculatePoints(){
    x_vals.clear();
    y_vals.clear();

    float max_time = max(option->timeS, option->timeD);

    for(short a = 1;a <= (max_time / refresh_rate); a++){
        float time = a * refresh_rate;
        double s = calculatePoint(time, *option->s);
        double d = calculatePoint(time, *option->d);

        std::vector<double> xy = mapData->getXY(s, d);
        cout << "s: " << s << " || d: " << d << " time = " << time << endl;
        // cout << "x: " << xy[0] << " || y: " << xy[1] << "\n" << endl;
        x_vals.push_back(xy[0]);
        y_vals.push_back(xy[1]);
    }
}