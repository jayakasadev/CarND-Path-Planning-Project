#include "trajectory.h"



void trajectory::generate(driver &driver, scores &score, map_data &mapData) {
    // reset the scores and predictions
    reset();

    // score each lane
    // get rid of lanes that are obviously bad choices
    // score.print();
    for(short a = 0; a < total_lanes; a++){
        // go through each lane
        // cancel out the obvious bad lanes
        calculate(driver, score, a);
        // cout << "lane: " << a << "\t";
    }
    cout << endl;
    printScores();
    short index = minScoreIndex();
    // cout << "cheapest index: " << index << endl;

    // generate the points here and convert from (s, d) to (x, y)
    for(short a = 0; a <= num_points; a++){
        double s = minimizers[index].predict(a * time_interval, true);
        double d = minimizers[index].predict(a * time_interval, false);
        vector<double> xy = mapData.getXY(s, d);
        cout << "s: " << s << " || d: " << d << " time = " << a * time_interval << endl;
        cout << "x: " << xy[0] << " || y: " << xy[1] << "\n" << endl;
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
}

void trajectory::calculate(driver &driver, scores &score, short &lane){
    cout << "CALCULATE: " << endl;
    minimizers[lane].resetViability();
    // go through each lane
    // cancel out the obvious bad lanes
    if(score.getLaneScore(lane) != OBSTRUCTION){
        minimizers[lane].calculate(driver.getS(), driver.getVelocityS(), driver.getAccelerationS(), driver.getS() + score.getDistanceFrontScore(lane), score.getVelocityScore(lane), 0, true); // s
        minimizers[lane].calculate(driver.getD(), driver.getVelocityD(), driver.getAccelerationD(), lane * 4 + 2, 0, 0, false); // d
        // if the lane is viable
        if(minimizers[lane].getViability()){
            costs[lane] = minimizers[lane].getCost(driver.getLane(), lane, driver.getVelocityS(), score);
        }
    }
}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}