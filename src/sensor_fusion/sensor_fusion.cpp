//
// Created by jay on 11/15/17.
//

#include "sensor_fusion.h"

// TODO make map threadsafe
void sensor_fusion::search(int lane, nlohmann::basic_json<> &sensor_fusion, int prev_size, driver &driver){
    for(int a = 0; a < sensor_fusion.size(); a++){
        // cout << "driver || s = " << driver.getS() << endl;
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
            // if(check_car_s < driver.getS()) cout << id << " || lane = " << lane << " || s = " << check_car_s << " || behind me" << endl;
            // cout << "others: " << others.size() << endl;
            if(others.find(id) != others.end()){
                // outdated
                if(others.at(id).updateTime() >= time_diff){
                    others.at(id).updateVehicle(vx, vy, x, y, d);
                }
                else{
                    // recent and valid
                    others.at(id).updateVehicle(vx, vy, x, y, d);
                }
            } else{
                other_vehicle otherVehicle;
                otherVehicle.updateVehicle(vx, vy, x, y, d);
                others[id] = otherVehicle;

            }

            double pred_s, pred_d, pred_velocity;
            others.at(id).predict(pred_s, pred_d, pred_velocity);

            double search_field_buffer = getSearch_field(lane, driver.getLane());
            double distance_to_target = check_car_s - (driver.getS() + buffer);

            // check if it is in the area of interest in its current lane
            // cout << "search_field_buffer: " << search_field_buffer << endl;
            // cout << id << " || lane = " << lane << " || s = " << check_car_s << " || me = " << driver.getS() << endl;
            // cout << "distance: " << distance_to_target << endl;
            if(abs(distance_to_target) <= search_field_buffer){
                if(lane == driver.getLane()){
                    lanes[lane] = FOLLOW;
                } else {
                    lanes[lane] = OBSTRUCTION; // mark lane as obstructed
                }
            }

            // vehicle is in front of me
            if(distance_to_target - buffer > 0 && distance_front[lane] > distance_to_target - buffer) {
                distance_front[lane] = distance_to_target - buffer;

                // only updated if lower than the current speed
                if(velocity[lane] > check_speed){
                    velocity[lane] = check_speed;
                }
            }
            //vehicle is behind me
            if(distance_to_target - buffer < 0 && distance_back[lane] < distance_to_target - buffer) {
                distance_back[lane] = distance_to_target - buffer;
            }

            short pred_lane = int(pred_d/4);
            // going to use the driver's state to predict his future lane
            short driver_lane = driver.getLane();

            switch (driver.getTurnType()){
                case TURN_LEFT:
                    driver_lane--;
                    break;
                case TURN_RIGHT:
                    driver_lane++;
                    break;
                default: // stay
                    break;
            }

            search_field_buffer = getSearch_field(pred_lane, driver_lane);
            distance_to_target = pred_s - (driver.predict() + buffer);

            // check if it is in the area of interest in its future lane
            // cout << "search_field_buffer: " << search_field_buffer << endl;
            // cout << id << " || pred_lane = " << pred_lane << " || pred_s = " << pred_s << " || me = " << driver.predict() << endl;
            // cout << "distance: " << distance_to_target << endl;
            if(abs(distance_to_target) <= search_field_buffer){
                if(pred_lane == driver_lane){
                    lanes[pred_lane] = FOLLOW;
                } else {
                    lanes[pred_lane] = OBSTRUCTION; // mark lane as obstructed
                }
            }

            // vehicle is in front of me
            if(distance_to_target - buffer > 0 && distance_front[pred_lane] > distance_to_target - buffer) {
                distance_front[pred_lane] = distance_to_target - buffer;
                // only updated if lower than the current speed
                if(velocity[pred_lane] > pred_velocity){
                    velocity[pred_lane] = pred_velocity;
                }
            }
            //vehicle is behind me
            if(distance_to_target - buffer < 0 && distance_back[pred_lane] < distance_to_target - buffer) {
                distance_back[pred_lane] = distance_to_target - buffer;
            }
        }
    }
}

void sensor_fusion::calculateCost(nlohmann::basic_json<> &sensor_fusion, int &prev_size, driver &driver){
    lanes.clear();
    velocity.clear();
    distance_back.clear();
    distance_front.clear();

    lanes = {OPEN, OPEN, OPEN};
    velocity = {speed_limit, speed_limit, speed_limit};
    distance_front = {spacing, spacing, spacing};
    distance_back = {-spacing, -spacing, -spacing};

    /*
    thread lane0([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });
    thread lane1([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });
    thread lane2([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });

    lane0.join();
    lane1.join();
    lane2.join();
     */

    search(0, sensor_fusion, prev_size, driver);
    search(1, sensor_fusion, prev_size, driver);
    search(2, sensor_fusion, prev_size, driver);

    // gonna return when all the threads are done

    cout << " lane# || lane_state || velocity || distance" << endl;
    for(int a = 0; a < 3; a++){
        cout << a << " || " << lanes[a] << " || " << velocity[a] << " || " << distance_front[a] << " || " << distance_back[a] << endl;
    }
}

// TODO create method to free up space in the map periodically