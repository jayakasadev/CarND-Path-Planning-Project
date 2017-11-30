#include "trajectory.h"

void trajectory::calculate(driver &driver, scores &score, short &lane){
    // go through each lane
    // cancel out the obvious bad lanes
    if(score.getLaneScore(lane) != OBSTRUCTION){
        minimizers[lane].calculate(driver.getS(), driver.getVelocityS(), driver.getAccelerationS(), score.getDistanceFrontScore(lane), score.getVelocityScore(lane), 0, true); // s
        minimizers[lane].calculate(driver.getD(), driver.getVelocityD(), driver.getAccelerationD(), lane * 4 + 2, 0, 0, false); // d
        // if the lane is viable
        if(minimizers[lane].getViability()){
            costs[lane] = minimizers[lane].getCost(driver.getLane(), lane, driver.getVelocityS(), score);
        }
    }
}

void trajectory::generate(int prev_size,nlohmann::basic_json<> &previous_path_x,
                          nlohmann::basic_json<> &previous_path_y, driver &driver, scores &score){
    // reset the scores
    resetScores();

    // score each lane
    // get rid of lanes that are obviously bad choices
    score.print();
    for(short a = 0; a < total_lanes; a++){
        // go through each lane
        // cancel out the obvious bad lanes
        calculate(driver, score, a);
    }
    printScores();
}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}