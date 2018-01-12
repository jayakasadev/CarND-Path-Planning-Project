//
// Created by jay on 12/5/17.
//

#include "trajectory_generator.h"

double trajectory_generator::calculatePoint(double &time, VectorXd &constants){
    return (*t).transpose() * constants;
}

void trajectory_generator::calculatePoints(trajectory_option &s_option, trajectory_option &d_option, short size){
    // cout << "trajectory_generator::calculatePoints\t" << size << endl;
    x_vals.clear();
    y_vals.clear();

    // float max_time = min(s_option.time, d_option.time);
    // short num_points_to_gen = max_time / refresh_rate;

    // double time = min(s_option.time, d_option.time);
    // cout << "\ttime: " << time << endl;
    // short num_points = (time / refresh_rate);
    double time;
    double s = 0;
    double d = 0;
    for(short a = 1; a <= num_points - size; a++){
        time = a * refresh_rate;
        *t << 1.0, time, pow(time, 2.0d) / 2.0d, pow(time, 3.0d) / 3.0d, pow(time, 4.0d) / 4.0d, pow(time, 5.0d) / 5.0d;
        s = (*t).transpose() * *s_option.vector;
        d = (*t).transpose() * *d_option.vector;

        // *t << 0.0d, 0.0d, 0.0d, 1.0d, time, pow(time, 2.0d) / 2.0d;
        // double jerk_s = (*t).transpose() * *s_option.vector;
        // double jerk_d = (*t).transpose() * *d_option.vector;

        std::vector<double> xy = mapData->getXY(s, d);

        double curr_diff = s - ps;
        if(curr_diff >= 2 * diff){
            outputfile << "WEIRD\t";
        }
        outputfile << "time: " << time;
        outputfile << "\t[ s: " << s << "\td: " << d << " ]";
        // outputfile << "\t[ x: " << xy[0] << "\ty: " << xy[1] << " ]";
        outputfile << "\tdiff_s: " << (s - ps) << "\tdiff_y: " << (d - pd)  << "\tv: " << sqrt(pow((xy[0] - px) / refresh_rate, 2) + pow((xy[1] - py) / refresh_rate, 2)) / mph_to_mps;
        outputfile << endl;
        x_vals.push_back(xy[0]);
        y_vals.push_back(xy[1]);
        ps = s;
        pd = d;
        px = xy[0];
        py = xy[1];
        diff = curr_diff;
    }

    sf = s;
    df = d;

    *t << 0.0d, 1.0d, time, pow(time, 2.0d) / 2.0d, pow(time, 3.0d) / 6.0d, pow(time, 4.0d) / 24.0d;
    sf_dot = (*t).transpose() * *s_option.vector;
    df_dot = (*t).transpose() * *d_option.vector;

    *t << 0.0d, 0.0d, 1.0d, time, pow(time, 2.0d) / 2.0d, pow(time, 3.0d) / 6.0d;
    sf_dot_dot = (*t).transpose() * *s_option.vector;
    df_dot_dot = (*t).transpose() * *d_option.vector;
}