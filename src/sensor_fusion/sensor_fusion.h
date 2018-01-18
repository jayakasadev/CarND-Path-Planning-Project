#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <unordered_map>

#include "../utilities/json.hpp"
#include "../vehicle/driver.h"
#include "../vehicle/traffic.h"
#include "../constants/sensor_fusion_constants.h"
#include "../detections/detections.h"

class sensorfusion {
private:
    std::unordered_map<short, traffic *> hashmap;
    driver * car;
    detections * detected;

    inline float getSearchRadius() {
        return search_radius + growth_rate * car->getVelocityS();
    }

public:
    sensorfusion(driver &car, detections &detected){
        this->car = &car;
        this->detected = &detected;
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
