//
// Created by jay on 11/15/17.
//

#include "../headers/trajectory.h"

void trajectory::generate(int prev_size,nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y, driver &driver,
                          const vector<lane_state> &lane_score, const vector<double> &velocity_score){

}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}