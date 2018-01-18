#include "trajectory.h"

std::vector<double> trajectory::getPoints(short n){
    if(!isInvalid()){
        throw new std::logic_error("This trajectory is invalid");
    }
    std::vector<double> out;
    if(!gotPoints(n)){
        if(type != trajectory_type::D){
            throw new std::logic_error("This trajectory does not have enough points");
        }
        // d trajectory and I passed the end, so just go straight
        for(short a = 0; a < n; a++){
            out.push_back(prev_point);
        }
    } else {
        Eigen::VectorXd t;
        for(short a = 0; a < n; a++){
            double curr_time = refresh_rate * count++;
            t << 1.0d, curr_time, pow(curr_time, 2.0d) / 2.0d, pow(curr_time, 3.0d) / 3.0d, pow(curr_time, 4.0d) / 4.0d, pow(curr_time, 5.0d) / 5.0d;
            prev_point = t.transpose() * *vector;
            out.push_back(prev_point);
        }
    }

    return out;
}

void trajectory::print(){
    std::cout << "trajectory\t vector: " << vector->transpose() << "\time: " << time << "\tscore: " << score
              << "\tcount: " << count << "\tinvalid: " << invalid << std::endl;
}