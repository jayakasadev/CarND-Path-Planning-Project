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
    driver *car;
    scores *values;

    inline float getSearchField(short lane) {
        float field = abs(lane - car->getLane()) * search_field_buffer + search_field_buffer;
        if(lane - car->getLane() == 0){
            return field;
        }
        return search_field_decay * field;
    }

    void setScore(double &s, double &d, double &velocity);

public:
    sensorfusion(driver &car, scores &scores){
        this->car = &car;
        this->values = &scores;
    }

    ~sensorfusion(){
        for(auto itr = hashmap.begin(); itr != hashmap.end(); itr++) {
            delete itr->second;
            hashmap.erase(itr->first);
        }
        hashmap.clear();
    }

    void predict(nlohmann::basic_json<> &sensor_fusion);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
