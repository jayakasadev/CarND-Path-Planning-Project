//
// Created by jay on 12/9/17.
//

#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H


#include "vehicle.h"
#include <chrono>
#include <vector>

#include "../constants/constants.h"

using namespace std::chrono;

class traffic : public vehicle {
private:
    double vx;
    double vy;
    std::chrono::high_resolution_clock::time_point last_Seen;

    inline void checkOutdated() {
        std::cout << "checkOutdated" << std::endl;
        long time = (high_resolution_clock::now() - last_Seen).count(); // TODO this is returning a large number that ends up being larger than the window i set
        std::cout << "time = " << time << std::endl;
        if (time > search_field_timelimit) { // last
            velocity_d = 0;
            velocity_s = 0;
            x = 0;
            y = 0;
            ps = 0;
            pd = 0;
            vx = 0;
            vy = 0;
            s = 0;
            d = 0;
            acceleration_s = 0;
            acceleration_d = 0;
            yaw = 0;
        }
    }

public:
    // constructor for sensor fusion
    traffic() {
        vx = 0;
        vy = 0;
        last_Seen = high_resolution_clock::now(); // update last seen time
    }

    ~traffic() {}

    void update(double x, double y, double vx, double vy, double s, double d);

    void print();
};


#endif //PATH_PLANNING_TRAFFIC_H
