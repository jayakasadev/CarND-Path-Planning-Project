//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <unordered_map>
#include "../map/map.h"
#include "../utilities/json.hpp"
#include "../behavior_planner/behavior_planner.h"
#include "../scores/scores.h"
#include "../vehicle/driver.h"
#include "../vehicle/traffic.h"
#include <thread>

using namespace std;

class sensorfusion {
private:
    std::unordered_map<short, traffic *> hashmap;
    scores *values;
    driver * car;
    map_data *mapData;

    inline float getSearchField(short lane, double car_lane) {
        float field = abs(lane - car_lane) * search_field_buffer + search_field_buffer;
        if(lane - car_lane == 0){
            return field;
        }
        return search_field_decay * field;
    }

    void setScore(double car_s, double car_d, double s, double d, double velocity);

public:
    sensorfusion(driver &car, scores &scores, map_data &mapData){
        this->car = &car;
        this->values = &scores;
        this->mapData = &mapData;
    }

    ~sensorfusion(){
        for(auto itr = hashmap.begin(); itr != hashmap.end(); itr++) {
            delete itr->second;
            hashmap.erase(itr->first);
        }
        hashmap.clear();
    }

    void predict(nlohmann::basic_json<> &sensor_fusion, nlohmann::basic_json<> &previous_path_x, nlohmann::basic_json<> &previous_path_y);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
