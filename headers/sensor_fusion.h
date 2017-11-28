//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <vector>
#include <math.h>
#include "constants.h"
#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <thread>
#include <map>

using namespace std;

class sensor_fusion {
private:

    void search(int lane, vector<vector<double>> &sensor_fusion, int prev_size, driver &driver);
    inline int min_element(float arr[], int size);

    vector<lane_state> lanes;
    vector<float> velocity;
    map<double, other_vehicle> others;

    inline short search_field(int lane, int curr_lane);

public:
    sensor_fusion(){}

    ~sensor_fusion(){}

    void calculateCost(vector<vector<double>> &sensor_fusion, int prev_size, driver &driver);

    inline vector<lane_state>* getLaneScore();
    inline vector<float>* getVelocityScore();
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
