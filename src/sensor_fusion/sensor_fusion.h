//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <vector>
#include <math.h>
// #include <iostream>
#include <algorithm>
#include <thread>
#include <map>

#include "../utilities/constants.h"
#include "../utilities/vehicle.h"
#include "../utilities/json.hpp"

using namespace std;

class sensor_fusion {
private:
    map<double, other_vehicle> others;

    void search(short lane, nlohmann::basic_json<> &sensor_fusion, int prev_size, driver &driver, scores &score);

    inline static double getSearch_field(int lane, int curr_lane) {
        double field = abs(lane - curr_lane) * search_field + search_field;
        if(lane - curr_lane == 0){
            return field;
        }
        return search_field_decay * field;
    }

    void setState(short &lane, double driver_s, double &s, double &velocity, short &driver_lane, scores &score);

public:
    sensor_fusion(){}

    ~sensor_fusion(){}

    void calculateCost(nlohmann::basic_json<> &sensor_fusion, int &prev_size, driver &driver, scores &score);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
