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

    void initialize(double s, double d, double speed);

    void update(double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot);

    void print();
};


#endif //PATH_PLANNING_DRIVER_H
