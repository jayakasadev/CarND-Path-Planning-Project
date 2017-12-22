//
// Created by jay on 12/5/17.
//

#include "trajectory_generator.h"

double trajectory_generator::calculatePoint(float &t, VectorXd &constants){
    Eigen::VectorXd x(6);
    x << 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
    return x.transpose() * constants;
}

void trajectory_generator::calculatePoints(trajectory_option &s_option, trajectory_option &d_option){
    x_vals.clear();
    y_vals.clear();

    // float max_time = max(s_option.time, d_option.time);

    for(short a = 1; a <= num_points_to_gen; a++){
        float time = a * refresh_rate;
        double s = calculatePoint(time, *s_option.vector);
        double d = calculatePoint(time, *d_option.vector);

        std::vector<double> xy = mapData->getXY(s, d);
        cout << "[ s: " << s << "\td: " << d << "\ttime = " << time << " ]" << endl;
        cout << "[ x: " << xy[0] << "\ty: " << xy[1] << " ]" << endl;
        x_vals.push_back(xy[0]);
        y_vals.push_back(xy[1]);
    }
}