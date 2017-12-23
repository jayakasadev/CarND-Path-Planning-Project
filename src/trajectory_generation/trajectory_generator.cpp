//
// Created by jay on 12/5/17.
//

#include "trajectory_generator.h"

double trajectory_generator::calculatePoint(double &time, VectorXd &constants){
    *t << 1.0, time, pow(time, 2), pow(time, 3), pow(time, 4), pow(time, 5);
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
        s = calculatePoint(time, *s_option.vector);
        d = calculatePoint(time, *d_option.vector);

        std::vector<double> xy = mapData->getXY(s, d);

        // cout << "[ s: " << s << "\td: " << d << "\ttime = " << time << " ]" << endl;
        // cout << "[ x: " << xy[0] << "\ty: " << xy[1] << " ]" << endl;
        x_vals.push_back(xy[0]);
        y_vals.push_back(xy[1]);
    }
    sf = s;
    df = d;

    *t << 0, 1, time, pow(time, 2) / 2, pow(time, 3) / 6, pow(time, 4) / 24;
    sf_dot = (*t).transpose() * *s_option.vector;
    df_dot = (*t).transpose() * *d_option.vector;

    *t << 0, 0, 1, time, pow(time, 2) / 2, pow(time, 3) / 6;
    sf_dot_dot = (*t).transpose() * *s_option.vector;
    df_dot_dot = (*t).transpose() * *d_option.vector;

    /*
    time -= refresh_rate;
    double sf_prev = calculatePoint(time, *s_option.vector);
    double df_prev = calculatePoint(time, *d_option.vector);

    sf_dot = (sf - sf_prev) / refresh_rate;
    df_dot = (df - df_prev) / refresh_rate;

    time -= refresh_rate;
    double sf_prev2 = calculatePoint(time, *s_option.vector);
    double df_prev2 = calculatePoint(time, *d_option.vector);


    double sf_dot_prev = (sf_prev - sf_prev2) / refresh_rate;
    double df_dot_prev = (df_prev - df_prev2) / refresh_rate;

    sf_dot_dot = (sf_dot - sf_dot_prev) / refresh_rate;
    df_dot_dot = (df_dot - df_dot_prev) / refresh_rate;
     */

    // cout << "sf: " << sf << "\tsf_dot: " << sf_dot << "\tsf_dot_dot: " << sf_dot_dot << endl;
    // cout << "df: " << df << "\tdf_dot: " << df_dot << "\tdf_dot_dot: " << df_dot_dot << endl;
}