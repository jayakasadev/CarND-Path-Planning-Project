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

class sensor_fusion {
private:
    std::unordered_map<short, others> hashmap;

    inline static float getSearchField(short lane, short curr_lane) {
        float field = abs(lane - curr_lane) * search_field_buffer + search_field_buffer;
        if(lane - curr_lane == 0){
            return field;
        }
        return search_field_decay * field;
    }

    void predictLane(short lane, nlohmann::basic_json<> &sensor_fusion, driver &driver, map_data &mapData, behavior_planner &planner);

public:
    sensor_fusion(){}

    ~sensor_fusion(){}

    void predict(nlohmann::basic_json<> &sensor_fusion, driver &driver, map_data &mapData, behavior_planner &planner);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
