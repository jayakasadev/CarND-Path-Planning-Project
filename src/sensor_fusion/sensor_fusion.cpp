//
// Created by jay on 12/5/17.
//

#include "sensor_fusion.h"

void sensorfusion::predict(nlohmann::basic_json<> &sensor_fusion, double size){
    // cout << "sensor_fusion::predict" << endl;
    // read everything into the hash_map and score the lane it is in

    for(short a  = 0; a < sensor_fusion.size(); a++){
        double id = sensor_fusion[a][0];
        // double x = sensor_fusion[a][1];
        // double y = sensor_fusion[a][2];
        double vx = sensor_fusion[a][3]; // lateral velocity
        double vy = sensor_fusion[a][4]; // longitudinal velocity
        double s = sensor_fusion[a][5]; // s value of the ath car
        double d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
        if(d >= 0 && d <= num_lanes * 4){ // only care for vehicles in lanes that i can go in
            // cout << id << " || vx = " << vx << " || vy = " << vy << endl;
            // cout << "map: " << hashmap.size() << endl;
            if(hashmap.find(id) == hashmap.end()){
                hashmap[id] = new traffic(); // add new element
            }
            hashmap.at(id)->update(vx, vy, s, d);

            // hashmap.at(id)->print();

            // score the vehicles here
            // cout << "velocity: " << velocity << endl;
            if(abs(car->getS() - s) <= spacing){
                this->setScore(car->getS(), car->getD(), s, d, hashmap.at(id)->getVelocityS());
            }
            double t1 = (- car->getVelocityS() + sqrt(pow(car->getVelocityS(), 2) - 4 * .5 * car->getAccelerationS() * (s - car->getS()))) / (car->getAccelerationS());
            double t2 = (- car->getVelocityS() - sqrt(pow(car->getVelocityS(), 2) - 4 * .5 * car->getAccelerationS() * (s - car->getS()))) / (car->getAccelerationS());

            double max_time = max(t1, t2);
            // cout << "time: " << max_time << endl;
            if(isnan(max_time)){
                max_time = 0;
            }

            for(double time = time_period; time <= max(max_time, double(time_period)); time += 0.1){
                if(abs(hashmap.at(id)->getPredictedS(time) - car->getS()) <= spacing){
                    // vector<double> predicted = hashmap.at(id)->getPredicted();
                    // cout << "s: " << hashmap.at(a)->getPredictedS(time) << " || d: " << hashmap.at(a)->getPredictedD(time) << endl;

                    setScore(car->getS(), car->getD(), hashmap.at(id)->getPredictedS(time),
                             hashmap.at(id)->getPredictedD(time), hashmap.at(id)->getPredictedVelocityS(time));
                }
            }
        }
    }

    values->printScores(); // print the values as a test for now
}