//
// Created by jay on 11/15/17.
//

#include "../headers/sensor_fusion.h"

void sensor_fusion::search(int lane, vector<vector<double>> &sensor_fusion, int prev_size, driver &driver){
    for(int a = 0; a < sensor_fusion.size(); a++){
        // check if car is in the desired lane
        double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        // checking if the car is in the range of my lane
        if(d < (4 + 4 * lane) && d > (4 * lane)){
            double id = sensor_fusion[a][0];
            double x = sensor_fusion[a][1];
            double y = sensor_fusion[a][2];
            double vx = sensor_fusion[a][3]; // lateral velocity
            double vy = sensor_fusion[a][4]; // longitudinal velocity
            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2)); // velocity
            double check_car_s = sensor_fusion[a][5]; // s value of the ath car
            other_vehicle* otherVehicle;
            if(others.find(id) != others.end()){
                *otherVehicle = others.at(id);
                // outdated
                if(otherVehicle->updateTime() >= time_diff){
                    others.at(id) = *new other_vehicle(vx, vy, x, y, d);
                }
                else{
                    // recent and valid
                    otherVehicle->updateVehicle(vx, vy, x, y, d);
                }
            } else{
                others.at(id) = *new other_vehicle(vx, vy, x, y, d);
            }

            double pred_s, pred_d, pred_velocity;
            otherVehicle->predict(pred_s, pred_d, pred_velocity, prev_size);

            // check if it is in the area of interest in its current lane
            if(abs(check_car_s - (driver.getS() + 5)) <= search_field(lane, driver.getLane())){
                lanes[lane] = OBSTRUCTION; // mark lane as obstructed

                // only updated if lower than the current speed
                if(velocity[lane] > check_speed){
                    velocity[lane] = check_speed;
                }
            }

            // check if it is in the area of interest in its future lane
            if(abs(pred_s - (driver.getS() + 5)) <= search_field((pred_d/4), driver.getLane())){
                if((pred_d/4) == driver.getLane() && pred_s > driver.getS()){
                    lanes[(pred_d/4)] = FOLLOW;
                }
                else{
                    lanes[(pred_d/4)] = OBSTRUCTION; // mark lane as obstructed
                }

                // only updated if lower than the current speed
                if(velocity[(pred_d/4)] > pred_velocity){
                    velocity[(pred_d/4)] = pred_velocity;
                }
            }
        }
    }
}

void sensor_fusion::calculateCost(vector<vector<double>> &sensor_fusion, int prev_size, driver &driver){
    lanes = {OPEN, OPEN, OPEN};
    velocity = {speed_limit, speed_limit, speed_limit};

    thread lane0(search(0, sensor_fusion, prev_size, driver));
    thread lane1(search(1, sensor_fusion, prev_size, driver));
    thread lane2(search(2, sensor_fusion, prev_size, driver));

    lane0.join();
    lane1.join();
    lane2.join();
    // gonna return when all the threads are done
}

inline vector<lane_state>* sensor_fusion::getLaneScore(){
    return &lanes;
}

inline vector<float>* sensor_fusion::getVelocityScore(){
    return &velocity;
}

inline short sensor_fusion::search_field(int lane, int curr_lane){
    return abs(lane - curr_lane) * 2.5 + 2.5;
}