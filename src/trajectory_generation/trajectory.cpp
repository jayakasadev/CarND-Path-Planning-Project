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
        // cout << "s: " << s << " || d: " << d << " time = " << a * time_interval << endl;
        // cout << "x: " << xy[0] << " || y: " << xy[1] << "\n" << endl;
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
}

void trajectory::calculate(driver &driver, scores &score, short &lane){
    cout << "trajectory::calculate: " << endl;
    minimizers[lane].resetViability();

    // go through each lane
    // cancel out the obvious bad lanes
    // if something is obstructing the current lane, predict for it too in case we want to avoid a collision
    if(score.getLaneScore(lane) != OBSTRUCTION || driver.getLane() == lane){
        double time = double(time_period);
        double d_x = score.getDistanceFrontScore(lane);
        double x = driver.getS();
        double xf = fmod(x + d_x, max_s); // keep from going off map
        double x_dot = driver.getVelocityS();
        double xf_dot = score.getVelocityScore(lane);
        double x_dot_dot = driver.getAccelerationS();
        calculateTimeS(x_dot, x_dot_dot, xf_dot, time);

        minimizers[lane].calculate(x, x_dot, x_dot_dot, xf, xf_dot, 0, time, true); // s

        x = driver.getD();
        x_dot = driver.getVelocityD();
        x_dot_dot = driver.getAccelerationD();
        xf = fmod(lane * 4 + 2, total_lanes * 4); // keep from going off map
        xf_dot = 0;
        calculateTimeD(x, xf, x_dot, x_dot_dot, xf_dot, time);
        minimizers[lane].calculate(x, x_dot, x_dot_dot, xf, xf_dot, 0, time, false); // d
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