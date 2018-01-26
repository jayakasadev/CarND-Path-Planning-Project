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

    friend std::ostream& operator <<(std::ostream& os, driver& obj){
        os << "driver:\t" << "[ s = " << obj.s << ", d = " << obj.d << ", velocity_s = " << obj.velocity_s
                          << ", acceleration_s = " << obj.acceleration_s << ", velocity_d = " << obj.velocity_d
                          << ", acceleration_d = " << obj.acceleration_d << " ]";
        return os;
    }
};


#endif //PATH_PLANNING_DRIVER_H
