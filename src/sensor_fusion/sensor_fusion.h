#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <unordered_map>

#include "../utilities/json.hpp"
#include "../scores/scores.h"
#include "../vehicle/driver.h"
#include "../vehicle/traffic.h"
#include "../constants/sensor_fusion_constants.h"

using namespace std;

class sensorfusion {
private:
    std::unordered_map<short, traffic *> hashmap;
    scores *values;
    driver * car;

    inline float getSearchRadius() {
        return search_radius + growth_rate * car->getVelocityS();
    }

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

    void predict(nlohmann::basic_json<> &sensor_fusion, double size);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
