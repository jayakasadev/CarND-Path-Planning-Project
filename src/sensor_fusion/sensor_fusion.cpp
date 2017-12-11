//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

// TODO setup scoring mechanism
void sensorfusion::setScore(double &s, double &d, double &velocity) {
    // cout << "sensorfusion::setScore" << endl;
    int lane = calculateLane(d);
    double distance = s - car->getS();
    double search_field = getSearchField(lane);
    // cout << "lane = " << lane << " || distance = " << distance << " || search_field = " << search_field << endl;
    if(lane == car->getLane()){
        if(distance <= search_field + search_field_buffer){
            // its in above the buffer line
            // check velocity for behavior
            if(velocity > 0 && velocity < values->getVelocity(lane)){ // obstacle is moving and below speed limit
                values->setFollow(lane);
                values->setVelocity(lane, velocity);
            } else if(velocity == 0){ // stop the vehicle if the obstacle is stationary
                values->setStop(lane);
                values->setVelocity(lane, 0);
            }
            values->setDistanceFront(lane, distance);
        } else if(distance >= -search_field + search_field_buffer){
            // it is below the buffer line
            if(lane == car->getLane()){
                if(velocity > 0 && velocity < values->getVelocity(lane)){ // obstacle is moving and below speed limit
                    values->setFollow(lane);
                    values->setVelocity(lane, velocity);
                } else if(velocity == 0){ // stop the vehicle if the obstacle is stationary
                    values->setStop(lane);
                    values->setVelocity(lane, 0);
                }
            } else {
                // it is obstructing the lane we want to go in
                values->setStop(lane);
                values->setVelocity(lane, 0);
                values->setDistanceBack(lane, distance);
            }
        }
    }
    // cout << "setScore Completed" << endl;
}

void sensorfusion::predict(nlohmann::basic_json<> &sensor_fusion){
    // cout << "sensor_fusion::predict" << endl;
    // read everything into the hash_map and score the lane it is in
    for(int a = 0; a < sensor_fusion.size(); a++){
        double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        if(d >= 0 && d <= num_lanes * 4){ // only care for vehicles in lanes that i can go in
            double id = sensor_fusion[a][0];
            double x = sensor_fusion[a][1];
            double y = sensor_fusion[a][2];
            double vx = sensor_fusion[a][3]; // lateral velocity
            double vy = sensor_fusion[a][4]; // longitudinal velocity
            double s = sensor_fusion[a][5]; // s value of the ath car

            // cout << id << " || vx = " << vx << " || vy = " << vy << endl;
            // cout << "map: " << hashmap.size() << endl;
            if(hashmap.find(id) == hashmap.end()){
                hashmap[id] = new traffic(); // add new element
            }
            hashmap.at(id)->update(x, y, vx, vy, s, d);

            // hashmap.at(id)->print();

            // score the vehicles here
            double velocity = hashmap.at(id)->getVelocityS();
            setScore(s, d, velocity);

            vector<double> predicted = hashmap.at(id)->getPredicted();
            // cout << "s: " << predicted[0] << " || d: " << predicted[1] << endl;
            velocity = hashmap.at(id)->getPredictedVelocityS();

            // TODO find the source of memory leak
            setScore(predicted[0], predicted[1], velocity);
        }
    }
    // cout << "printing scores" << endl;
    values->printScores(); // print the values as a test for now
}