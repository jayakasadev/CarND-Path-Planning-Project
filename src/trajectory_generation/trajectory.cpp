#include "trajectory.h"

void trajectory::generate(int prev_size,nlohmann::basic_json<> &previous_path_x,
                          nlohmann::basic_json<> &previous_path_y, driver &driver, scores &score){
    // calculate trajectory
    // score.print();
}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}