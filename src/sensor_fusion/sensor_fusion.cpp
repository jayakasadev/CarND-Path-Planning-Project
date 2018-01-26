#include "sensor_fusion.h"

void sensorfusion::predict(nlohmann::basic_json<> &sensor_fusion, double size){
    // std::cout << "sensor_fusion::predict" << std::endl;
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
            // std::cout << id << " || vx = " << vx << " || vy = " << vy << std::endl;
            // std::cout << "map: " << hashmap.size() << std::endl;
            if(hashmap.find(id) == hashmap.end()){
                hashmap[id] = std::make_shared<traffic>(); // add new element
            }
            hashmap.at(id)->update(vx, vy, s, d);

            // is the vehicle near me?
            if(abs(car->getS() - s) <= search_radius_at_curr_v){
                // std::cout << "sensor_fusion predict addr: "  << hashmap.at(id).get() << std::endl;
                detected->add(hashmap.at(id));
            } else {
                // check if the vehicle passes by my car at any point in the next second
                for(double time = refresh_rate; time <= time_period; time += refresh_rate){
                    if(abs(hashmap.at(id)->getPredictedS(time) - car->getS()) <= search_radius_at_curr_v){
                        // std::cout << "sensor_fusion predict addr: "  << hashmap.at(id).get() << std::endl;
                        detected->add(hashmap.at(id));
                        break;
                    }
                }
            }
        }
    }

    /*
    if(detected->size() > 0)
        std::cout << *detected.get() << std::endl;
    */
}