#include "mode.h"

double regular_mode::time_diff_cost(double time){
    return logistic((time - time_period) / time_period);
}

double regular_mode::efficiency_cost(Eigen::VectorXd &s, double time, double targ_s){
    if(s.size() != 6){
        throw std::invalid_argument("vector contain must be 6 elements");
    }

    Eigen::VectorXd t;
    t << 1, time, pow(time, 2) / 2, pow(time, 3) / 6, pow(time, 4) / 24, pow(time, 5) / 120;
    double avg_v = s.transpose() * t;

    return logistic(2 * float(targ_s - avg_v) / avg_v);
}

// drive regular_mode dependent behavior

bool regular_mode::collision_cost(Eigen::VectorXd &s, Eigen::VectorXd &d, double time){
    if(d.size() != 6 || s.size() != 6){
        throw std::invalid_argument("s and d must contain 6 elements");
    }

    Eigen::VectorXd t;
    short points = (time / refresh_rate);
    for(short a = 0; a < points; a++){
        double t_interval = refresh_rate * a;
        t << 1, t_interval, pow(t_interval, 2) / 2, pow(t_interval, 3) / 6, pow(t_interval, 4) / 24, pow(t_interval, 5) / 120;
        double calc_d = d.transpose() * t;
        if(calc_d <= lane_edge_buffer || calc_d >= (num_lanes * 4 - lane_edge_buffer)){
            throw std::logic_error("trajectory will take vehicle off road or into oncoming traffic");
        }
        double calc_s = s.transpose() * t;
        if(calc_s < s[0]){
            throw std::logic_error("trajectory will cause vehicle to drive in the wrong direction");
        }

        // TODO implement method to compare vehicles for collision
    }
    return true;
}

double regular_mode::avg_jerk_cost(Eigen::VectorXd &s, double time){
    if(s.size() != 6){
        throw std::invalid_argument("vector contain must be 6 elements");
    }

    Eigen::VectorXd t;
    short points = (time / refresh_rate);
    double avg_jerk = 0;
    for(short a = 0; a < points; a++){
        double t_interval = refresh_rate * a;
        t << 0, 0, 0, 1, t_interval, pow(t_interval, 2) / 2;
        double local_jerk = abs(s.transpose() * t);
        if(local_jerk >= max_jerk){
            throw std::logic_error("local jerk exceeds max jerk");
        }
        avg_jerk += local_jerk;
    }
    avg_jerk /= points;
    return logistic(avg_jerk / desired_jerk);
}

double regular_mode::avg_acc_cost(Eigen::VectorXd &s, double time){
    if(s.size() != 6){
        throw std::invalid_argument("vector contain must be 6 elements");
    }

    Eigen::VectorXd t;
    short points = (time / refresh_rate);
    double avg_acc = 0;
    for(short a = 0; a < points; a++){
        double t_interval = refresh_rate * a;
        t << 0, 0, 1, t_interval, pow(t_interval, 2) / 2, pow(t_interval, 3) / 6;
        double local_acc = abs(s.transpose() * t);
        if(abs(local_acc) >= max_acceleration){
            throw std::logic_error("local acceleration exceeds max acceleration");
        }
        avg_acc += local_acc;
    }
    avg_acc /= points;
    return logistic(avg_acc / desired_acceleration);
}