//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <vector>
#include <math.h>
#include "../utilities/constants.h"
#include "../utilities/vehicle.h"
#include "../utilities/json.hpp"
#include <iostream>
#include <algorithm>
#include <thread>
#include <map>

using namespace std;

class sensor_fusion {
private:

    void search(int lane, nlohmann::basic_json<> &sensor_fusion, int prev_size, driver &driver);
    inline int min_element(float arr[], int size);

    vector<lane_state> lanes;
    vector<double> velocity;
    vector<double> distance_front;
    vector<double> distance_back;
    map<double, other_vehicle> others;

    inline static double getSearch_field(int lane, int curr_lane) {
        double field = abs(lane - curr_lane) * search_field + search_field;
        if(lane - curr_lane == 0){
            return field;
        }
        return search_field_decay * field;
    }

public:
    sensor_fusion(){}

    ~sensor_fusion(){}

    void calculateCost(nlohmann::basic_json<> &sensor_fusion, int &prev_size, driver &driver);

    inline vector<lane_state> getLaneScore(){
        return lanes;
    }
    inline vector<double> getVelocityScore(){
        return velocity;
    }
    inline vector<double> getDistanceFrontScore(){
        return distance_front;
    }
    inline vector<double> getDistanceBackScore(){
        return distance_back;
    }
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
