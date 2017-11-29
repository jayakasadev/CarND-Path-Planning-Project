//
// Created by jay on 11/15/17.
//

#include "sensor_fusion.h"

// TODO make map threadsafe
void sensor_fusion::search(short lane, nlohmann::basic_json<> &sensor_fusion, int prev_size, driver &driver, scores &score){
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

            short driver_lane = driver.getLane();

            setState(lane, driver, check_car_s, check_speed, driver_lane, score); // state setting for current time

            double pred_s, pred_d, pred_velocity;
            others.at(id).predict(pred_s, pred_d, pred_velocity);

            short pred_lane = int(pred_d/4);
            // going to use the driver's state to predict his future lane
            if(driver.getDesiredLane() < driver.getLane()) {
                driver_lane--;
            } else if(driver.getDesiredLane() > driver.getLane()) {
                driver_lane++;
            }

            setState(pred_lane, driver, pred_s, pred_velocity, driver_lane, score); // state setting for 0.02 seconds in future
        }
    }
}

void sensor_fusion::calculateCost(nlohmann::basic_json<> &sensor_fusion, int &prev_size, driver &driver, scores &score){
    score.clear();
    score.setup();
    /*
    thread lane0([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });
    thread lane1([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });
    thread lane2([this, &sensor_fusion, &prev_size, &driver]{ this->search(0, sensor_fusion, prev_size, driver); });

    lane0.join();
    lane1.join();
    lane2.join();
     */

    search(0, sensor_fusion, prev_size, driver, score);
    search(1, sensor_fusion, prev_size, driver, score);
    search(2, sensor_fusion, prev_size, driver, score);

    // gonna return when all the threads are done
    // score.print();
}

void sensor_fusion::setState(short &lane, driver &driver, double &s, double &velocity, short &driver_lane, scores &score){
    double search_field_buffer = getSearch_field(lane, driver_lane);
    double distance_to_target = s - (driver.getS() + buffer);

    // check if it is in the area of interest in its current lane
    // cout << "search_field_buffer: " << search_field_buffer << endl;
    // cout << id << " || lane = " << lane << " || s = " << check_car_s << " || me = " << driver.getS() << endl;
    // cout << "distance: " << distance_to_target << endl;
    if(abs(distance_to_target) <= search_field_buffer){
        if(lane == driver_lane){
            score.setLaneFollow(lane);
        } else {
            score.setLaneObstructed(lane);// mark lane as obstructed
        }
    }

    // vehicle is in front of me
    if(distance_to_target - buffer > 0 && score.getDistanceFrontScore(lane) > distance_to_target - buffer) {
        score.setDistanceFrontScore(lane, distance_to_target - buffer);

        // only updated if lower than the current speed
        if(score.getVelocityScore(lane) > velocity){
            score.setVelocityScore(lane, velocity);
        }
    }
    //vehicle is behind me
    if(distance_to_target - buffer < 0 && score.getDistanceBackScore(lane) < distance_to_target - buffer) {
        score.setDistanceBackScore(lane, distance_to_target - buffer);
    }
}

// TODO create method to free up space in the map periodically