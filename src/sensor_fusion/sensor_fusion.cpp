//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

void sensorfusion::setScore(double &s, double &d, double &velocity) {
    // cout << "sensorfusion::setScore" << endl;
    int lane = calculateLane(d);
    double distance = s - car->getS();
    double search_field = getSearchField(lane);

    if(distance <= values->getDistanceFront(lane) && distance > 0){ // it is less than 22 meters away
        // cout << "lane = " << lane << " || distance = " << distance << " || search_field = " << search_field << endl;
        // cout << "front of me: " << (distance <= values->getDistanceFront(lane))  << " worried? " << (distance <= (search_field + search_field_buffer)) << endl;
        // the vehicle is in front
        if(distance <= (search_field + search_field_buffer) || driveMode == ECONOMY){ // the vehicle is about 2 cars away or i want the most economic drive
            // set behavior and save velocity and distance in front
            if(lane == car->getLane()){
                // only time follow is possible is if vehicle is in my lane
                if(velocity > 0){
                    values->setFollow(lane);
                } else{
                    values->setStop(lane);
                }
                values->setVelocity(lane, velocity);
            } else {
                values->setStop(lane);
                values->setVelocity(lane, 0);
            }
            // cout << "behavior: " << values->getBehavior(lane) << " velocity: " << values->getVelocity(lane) << endl;
        }
        values->setDistanceFront(lane, distance);
        // cout << "distance front: " << values->getDistanceFront(lane) << endl;
    } else if(distance >= values->getDistanceBack(lane) && distance <= 0){
        // cout << "lane = " << lane << " || distance = " << distance << " || search_field = " << search_field << endl;
        // cout << "behind me: " << (distance >= values->getDistanceBack(lane)) << " worried? " << (distance >= (-search_field + search_field_buffer)) << endl;
        // vehicle is behind
        if(distance >= (-search_field + search_field_buffer)){ // vehicle is next to driver
            // set to stop and save velocity and distance in front
            values->setStop(lane);
            values->setVelocity(lane, 0);
            // cout << "behavior: " << values->getBehavior(lane) << " velocity: " << values->getVelocity(lane) << endl;
        }
        values->setDistanceBack(lane, distance);
        // cout << "distance back: " << values->getDistanceBack(lane) << endl;
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

            setScore(predicted[0], predicted[1], velocity);
        }
    }
    // cout << "printing scores" << endl;
    // thread sf([this, ref(values)]{values->printScores();});
    // values->printScores(); // print the values as a test for now
}