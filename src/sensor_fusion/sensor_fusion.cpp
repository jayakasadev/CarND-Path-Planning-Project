//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

void sensorfusion::setScore(double car_s, double car_d, double s, double d, double velocity) {
    // cout << "sensorfusion::setScore" << endl;
    short lane = calculateLane(d);
    short car_lane = calculateLane(car_d);
    double distance = s - car_s;
    double search_field = getSearchField(lane, car_lane);

    if(distance <= values->getDistanceFront(lane) && distance > 0){ // it is less than 22 meters away
        // cout << "lane = " << lane << " || distance = " << distance << " || search_field = " << search_field << endl;
        // cout << "front of me: " << (distance <= values->getDistanceFront(lane))  << " worried? " << (distance <= (search_field + search_field_buffer)) << endl;
        // the vehicle is in front
        if(distance <= (search_field + search_field_buffer) || driveMode == ECONOMY){ // the vehicle is about 2 cars away or i want the most economic drive
            // set behavior and save velocity and distance in front
            if(lane == car_lane){
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

void sensorfusion::predict(nlohmann::basic_json<> &sensor_fusion, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y){
    // cout << "sensor_fusion::predict" << endl;
    // read everything into the hash_map and score the lane it is in

    for(short a  = 0; a < sensor_fusion.size(); a++){
        double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        double id = sensor_fusion[a][0];
        double x = sensor_fusion[a][1];
        double y = sensor_fusion[a][2];
        double vx = sensor_fusion[a][3]; // lateral velocity
        double vy = sensor_fusion[a][4]; // longitudinal velocity
        double s = sensor_fusion[a][5]; // s value of the ath car
        if(d >= 0 && d <= num_lanes * 4){ // only care for vehicles in lanes that i can go in
            // cout << id << " || vx = " << vx << " || vy = " << vy << endl;
            // cout << "map: " << hashmap.size() << endl;
            if(hashmap.find(id) == hashmap.end()){
                hashmap[id] = new traffic(); // add new element
            }
            hashmap.at(id)->update(x, y, vx, vy, s, d);

            // hashmap.at(id)->print();

            // score the vehicles here
            double velocity = hashmap.at(id)->getVelocityS();
            // cout << "velocity: " << velocity << endl;
            this->setScore(car->getS(), car->getD(), s, d, velocity);

            // vector<double> predicted = hashmap.at(id)->getPredicted();
            // cout << "s: " << hashmap.at(a)->getPredictedS(time_period) << " || d: " << hashmap.at(a)->getPredictedD(time_period) << endl;
            velocity = hashmap.at(id)->getPredictedVelocityS(time_period);

            setScore(car->getS(), car->getD(), hashmap.at(id)->getPredictedS(time_period), hashmap.at(id)->getPredictedD(time_period), velocity);
        }
    }

    /*
    if(previous_path_x.size() > 0){ // do more predictions on my path if there is a path
        vector<future<void>> options;

        for(short a = 0; a < sensor_fusion.size(); a++){
            double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
            double id = sensor_fusion[a][0];
            double x = sensor_fusion[a][1];
            double y = sensor_fusion[a][2];
            double vx = sensor_fusion[a][3]; // lateral velocity
            double vy = sensor_fusion[a][4]; // longitudinal velocity
            double s = sensor_fusion[a][5]; // s value of the ath car
            options.push_back(async(launch::async, [this, d, id, x, y ,vx, vy, s, &previous_path_x, &previous_path_y]{
                if(d >= 0 && d <= num_lanes * 4){ // only care for vehicles in lanes that i can go in

                    // cout << id << " || vx = " << vx << " || vy = " << vy << endl;
                    // cout << "map: " << hashmap.size() << endl;
                    if(hashmap.find(id) == hashmap.end()){
                        hashmap[id] = new traffic(); // add new element
                    }
                    hashmap.at(id)->update(x, y, vx, vy, s, d);

                    // hashmap.at(id)->print();

                    for(short b = 1; b < previous_path_x.size(); b++){

                        // score the vehicles here
                        double velocity = hashmap.at(id)->getVelocityS();
                        // cout << "velocity: " << velocity << endl;
                        double x = previous_path_x[b], y = previous_path_y[b], px = previous_path_x[b-1], py = previous_path_y[b-1];
                        // cout << "previous_path_x: " << x;
                        // cout << "\tprevious_path_y: " << y << endl;
                        vector<double> s_d = mapData->getFrenet(x, y, calculateYaw(y, py, x, px));
                        this->setScore(s_d[0], s_d[1], s, d, velocity);

                        // vector<double> predicted = hashmap.at(id)->getPredicted();
                        double time = b * refresh_rate;
                        // cout << "s: " << hashmap.at(id)->getPredictedS(time) << " || d: " << hashmap.at(id)->getPredictedD(time) << endl;
                        velocity = hashmap.at(id)->getPredictedVelocityS(time);

                        setScore(s_d[0], s_d[1], hashmap.at(id)->getPredictedS(time), hashmap.at(id)->getPredictedD(time), velocity);

                    }
                }
            }));
        }
        // cout << "options size: " << options.size() << endl;
        // thread sf([this, ref(values)]{values->printScores();});

        for(short a = 0; a < options.size(); a++){
            options[a].get();
        }
    }
     */

    // values->printScores(); // print the values as a test for now
}