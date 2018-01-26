//
// Created by jay on 12/9/17.
//

#ifndef PATH_PLANNING_DRIVER_H
#define PATH_PLANNING_DRIVER_H

#include "vehicle.h"

class driver : public vehicle {
public:
    // constructor for driver
    driver(){
        // std::cout << "driver constructor" << std::endl;
    }

    ~driver(){
        std::cout << "driver destructor" << std::endl;
    }

    void initialize(double s, double d, double speed);

    void update(double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot);

    virtual std::ostream& print(std::ostream& os) const {
        return os << "driver\t";
    }
};


#endif //PATH_PLANNING_DRIVER_H
