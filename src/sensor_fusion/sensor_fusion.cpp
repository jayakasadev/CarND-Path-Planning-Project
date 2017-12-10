//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

void sensorfusion::setScore(double s, double d, double velocity) {
    // cout << "sensorfusion::setScore" << endl;
    int lane = calculateLane(d);
    double distance = car->getS() - s;
    double search_field = getSearchField(lane);
    // cout << "lane = " << lane << " || distance = " << distance << " || search_field = " << search_field << endl;
    if(distance > 0){ // within the search field
        // cout << "in front" << endl;
        // vehicle in front
        // check if it is in my search field
        // mark lane behavior
        // note velocity
        if(lane == car->getLane()){
            if(distance <= search_field + 5) {
                if(velocity > 0 && velocity < max_velocity_mps){
                    values->setBehavior(lane, FOLLOW);
                }
                else if(velocity == 0){
                    values->setBehavior(lane, STOP);
                }
                values->setVelocity(lane, velocity);
            }
        }


    }
    else {
        // cout << "behind" << endl;
        // vehicle is next to or behind me
        // check if it is in my search field
        // note distance_back
        // note distance_front
        // cout << "setBehavior and setVelocity" << endl;
        if(distance >= -search_field + 5) {
            values->setBehavior(lane, STOP);
            values->setVelocity(lane, 0);
        }
        // cout << "getDistanceBack" << endl;
        if(values->getDistanceBack(lane) < distance){
            // cout << "setDistanceBack" << endl;
            values->setDistanceBack(lane, distance);
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

            cout << id << " || s = " << s << " || d = " << d << endl;
            cout << "map: " << hashmap.size() << endl;
            if(hashmap.find(id) != hashmap.end()){
                hashmap.at(id).update(x, y, vx, vy, s, d);
            } else{
                traffic* vehicle = new traffic;
                vehicle->update(x, y, vx, vy, s, d);
                hashmap[id] = *vehicle;
            }
            // score the vehicles here
            double velocity = hashmap.at(id).getVelocityS();
            setScore(s, d, velocity);

            vector<double> predicted = hashmap.at(id).getPredicted();
            cout << "s: " << predicted[0] << " || d: " << predicted[1] << endl;
            velocity = hashmap.at(id).getPredictedVelocityS();
            setScore(predicted[0], predicted[1], velocity);

            hashmap.at(id).print();
        }
    }
    cout << "printing scores" << endl;
    // values->printScores(); // print the values as a test for now
}