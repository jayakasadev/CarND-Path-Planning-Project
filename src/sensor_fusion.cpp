//
// Created by jay on 11/15/17.
//

#include "sensor_fusion.h"

void sensor_fusion::search(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s, bool side, double &speed, float &cost){
    double front_car = car_s + 30;
    double back_car = car_s - 7;

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
                // cout << "\tOBSTACLE lane:" << lane << endl;
                // too_close = true; // flag used to make incremental changes to the speed
                cout << "lane: " << lane << " check_car_s: " << check_car_s << " car_s: " << car_s << " check_speed: " << check_speed << endl;
                speed = check_speed;
                front_car = check_car_s;
                // cout << "speed: " << speed << endl;
                if(!side){ // if im not trying to check the sides of the car
                    break; // added because there is no point continuing the search when i know that I have to slow down
                }
            }
            if(side){
                // check the sides as well because the car is not in this lane and may want to consider turning into a new lane
                if((check_car_s < car_s) && ((car_s - check_car_s) < 7)) { // checking the side/back
                    cout << "lane: " << lane << " check_car_s: " << check_car_s << " car_s: " << car_s << " check_speed: " << check_speed << endl;
                    back_car = check_car_s;
                }
            }
        }
    }

    if(side){
        if(front_car - back_car >= 37){ // if there is enough space to go
            cost = - speed + 100;
        }
    }else{
        cost = - 1.005 * speed + 100;
    }
    // cout << "lane: " << lane << " cost: " << cost << endl;
}

void sensor_fusion::calculateCost(vector<vector<double>> sensor_fusion, int prev_size, double car_s, short lane){
    double speed[3] = {speed_limit, speed_limit, speed_limit};
    float cost[3] = {100, 100, 100};
    if((lane - 1) >= 0)
        search(lane - 1, sensor_fusion, prev_size, car_s, true, speed[lane - 1], cost[lane - 1]); // check left lane
    search(lane, sensor_fusion, prev_size, car_s, false, speed[lane], cost[lane]); // check current lane
    if((lane + 1) <= 2)
        search(lane + 1, sensor_fusion, prev_size, car_s, true, speed[lane + 1], cost[lane + 1]); // check right lane
    // cout << "elements: " << sizeof(cost) / sizeof(cost[0]) << endl;
    int index = min_element(cost, 3);
    cout << "cost: [ " << cost[0] << ", " << cost[1] << ", " << cost[2] << " ]"<< endl;
    cout << "min element at: " << index << endl;
    this->speed = speed[index];
    this->lane = index;
}

int sensor_fusion::getLane(){
    return lane;
}

double sensor_fusion::getSpeed(){
    return speed;
}

inline int sensor_fusion::min_element(float arr[], int size){
    int index = 0;
    // cout << "elements: " << size << endl;
    for(int a = 1; a < size; a++){
        if(arr[index] > arr[a]){
            index = a;
        }
    }
    return index;
}