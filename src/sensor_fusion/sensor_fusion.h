//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <unordered_map>
#include "../utilities/vehicle.h"
#include "../map/map.h"
#include "../utilities/json.hpp"
#include "../behavior_planner/behavior_planner.h"
#include "../scores/scores.h"

class sensor_fusion {
private:
    std::unordered_map<short, others> hashmap;
    map_data *mapData;
    driver *car;
    scores *values;

    inline static float getSearchField(short lane, short curr_lane) {
        float field = abs(lane - curr_lane) * search_field_buffer + search_field_buffer;
        if(lane - curr_lane == 0){
            return field;
        }
        return search_field_decay * field;
    }

    void predictLane(short lane, nlohmann::basic_json<> &sensor_fusion);

    void setScoreForLane(short lane, double);

public:
    sensor_fusion(driver &car, map_data &mapData, scores &scores){
        this->car = &car;
        this->mapData = &mapData;
        this->values = &scores;
    }

    ~sensor_fusion(){}

    void predict(nlohmann::basic_json<> &sensor_fusion);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
