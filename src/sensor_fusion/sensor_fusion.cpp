#include "sensor_fusion.h"

void sensorfusion::predict(nlohmann::basic_json<> &sensor_fusion, double size){
    // cout << "sensor_fusion::predict" << endl;
    // read everything into the hash_map and score the lane it is in
    double search_radius_at_curr_v = getSearchRadius();
    // std::cout << "search radius: " << search_radius_at_curr_v << std::endl;
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

            // is the vehicle near me?
            if(abs(car->getS() - s) <= search_radius_at_curr_v){
                detected->add(*hashmap.at(id));
            }

            // check if the vehicle passes by my car at any point in the next second
            for(double time = refresh_rate; time <= time_period; time += refresh_rate){
                if(abs(hashmap.at(id)->getPredictedS(time) - car->getS()) <= search_radius_at_curr_v){
                    detected->add(*hashmap.at(id));
                }
            }
        }
    }

    // detected->print();
}