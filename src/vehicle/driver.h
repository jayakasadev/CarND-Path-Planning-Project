//
// Created by jay on 12/9/17.
//

#ifndef PATH_PLANNING_DRIVER_H
#define PATH_PLANNING_DRIVER_H

#include "vehicle.h"
#include "../constants/constants.h"

class driver : public vehicle {
public:
    // constructor for driver
    driver(){}

    ~driver(){}

    void update(double x, double y, double s, double d, double yaw, double speed);

    void print();
};


#endif //PATH_PLANNING_DRIVER_H
