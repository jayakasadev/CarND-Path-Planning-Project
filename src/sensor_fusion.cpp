//
// Created by jay on 11/15/17.
//

#include "sensor_fusion.h"

float sensor_fusion::search(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s, bool side, double &speed, float &cost){
    // find ref_v to use
    for(int a = 0; a < sensor_fusion.size(); a++){
        // car is in my lane
        // looking at the ath car on the road
        float d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        // cout << "a: " << a << " d: " << d << endl;
        if(d < (4 + 4 * lane) && d > (4 * lane)){ // checking if the car is in the range of my lane
            double vx = sensor_fusion[a][3]; // lateral velocity
            double vy = sensor_fusion[a][4]; // longitudinal velocity
            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2)); // velocity
            double check_car_s = sensor_fusion[a][5]; // s value of the ath car
            // cout << "\tcheck_car_s: " << check_car_s << endl;

            // if using previous points can project s value out
            check_car_s += ((double) prev_size * time_interval * check_speed);// predicting where the car will be (.02 * prev points) seconds in the future
            // cout << "\tfuture check_car_s: " << check_car_s << endl;
            // cout << "\tcar_s: " << car_s << endl;

            // check s values greater than mine and s gap
            if((check_car_s > car_s) && ((check_car_s - car_s) < 30)) { // check if the car will be within 30m of my car in the future
                // do some logic here, lower reference velocity so we don't crash into the car in front of us, could also flag to try to change lanes
                // ref_vel = 29.5; // mph // set speed change
                // cout << "\tOBSTACLE" << endl;
                // too_close = true; // flag used to make incremental changes to the speed
                speed = check_speed;
                // cout << "speed: " << speed << endl;
                break; // added because there is no point continuing the search when i know that I have to slow down
            }
        }
    }
}

void sensor_fusion::calculateCost(vector<vector<double>> sensor_fusion, int prev_size, double car_s){
    double speed = speed_limit;
    float cost = 0;
    search(1, sensor_fusion, prev_size, car_s, false, speed, cost);
    this->speed = speed;
    this->lane = 1;
}

int sensor_fusion::getLane(){
    return lane;
}

double sensor_fusion::getSpeed(){
    return speed;
}