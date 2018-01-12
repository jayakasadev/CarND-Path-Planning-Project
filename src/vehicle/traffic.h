//
// Created by jay on 12/9/17.
//

#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <chrono>
#include <vector>

#include "vehicle.h"
#include "../constants/sensor_fusion_constants.h"


using namespace std::chrono;

class traffic : public vehicle {
private:
    std::chrono::high_resolution_clock::time_point last_Seen;

    bool first;

    inline void checkOutdated() {
        // std::cout << "checkOutdated" << std::endl;
        duration<double, std::milli> time_span = high_resolution_clock::now() - last_Seen;
        short time = time_span.count();
        // std::cout << "time = " << time << std::endl;
        if (time > time_limit) { // last
            velocity_d = 0;
            velocity_s = 0;
            ps = 0;
            pd = 0;
            s = 0;
            d = 0;
            acceleration_s = 0;
            acceleration_d = 0;
        }
    }

public:
    // constructor for sensor fusion
    traffic() {
        last_Seen = high_resolution_clock::now(); // update last seen time
        first = true;
    }

    ~traffic() {}

    void update(double vx, double vy, double s, double d);

    void print();
};


#endif //PATH_PLANNING_TRAFFIC_H
