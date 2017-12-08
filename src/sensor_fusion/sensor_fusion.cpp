//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

void sensor_fusion::predictLane(short lane, nlohmann::basic_json<> &sensor_fusion){
    for(int a = 0; a < sensor_fusion.size(); a++){
        // cout << "driver || s = " << driver.getS() << endl;
        // check if car is in the desired lane
        double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        // checking if the car is in the range of my lane
        if(d < (4 + 4 * lane) && d > (4 * lane)){
            short id = sensor_fusion[a][0];
            float x = sensor_fusion[a][1];
            float y = sensor_fusion[a][2];
            float vx = sensor_fusion[a][3]; // lateral velocity
            float vy = sensor_fusion[a][4]; // longitudinal velocity
            float check_speed = sqrt(pow(vx, 2) + pow(vy, 2)); // velocity
            float check_car_s = sensor_fusion[a][5]; // s value of the ath car
            // if(check_car_s < driver.getS()) cout << id << " || lane = " << lane << " || s = " << check_car_s << " || behind me" << endl;
            // cout << "others: " << others.size() << endl;
            if(hashmap.find(id) != hashmap.end()){
                hashmap.at(id).update(x, y, vx, vy, check_car_s, d);
            } else{
                others otherVehicle;
                otherVehicle.update(x, y, vx, vy, check_car_s, d);
                hashmap[id] = otherVehicle;

            }
        }
    }
}

void sensor_fusion::predict(nlohmann::basic_json<> &sensor_fusion){
    for(short a = 0; a < num_lanes; a++){
        predictLane(a, sensor_fusion);
    }

    values->printScores();
}