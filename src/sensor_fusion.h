//
// Created by jay on 11/15/17.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <vector>
#include <math.h>
#include "constants.h"
#include <iostream>
#include <algorithm>

using namespace std;

class sensor_fusion {
private:
    int lane;
    double speed;

    void search(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s, bool side, double &speed, float &cost);
    inline int min_element(float arr[], int size);

public:
    sensor_fusion(){}

    ~sensor_fusion(){}

    int getLane();

    double getSpeed();

    void calculateCost(vector<vector<double>> sensor_fusion, int prev_size, double car_s, short lane);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
