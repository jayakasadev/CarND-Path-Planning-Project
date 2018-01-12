//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>
#include "../utilities/utilities.h"

struct vehicle{
protected:

    double s;
    double d;

    double velocity_s; // in m/s
    double velocity_d; // in m/s

    double acceleration_s; // in m/s^2
    double acceleration_d; // in m/s^2

public:
    vehicle(){
        s = 0;
        d = 0;
        velocity_s = 0;
        velocity_d = 0;
        acceleration_s = 0;
        acceleration_d = 0;
    }
    ~vehicle(){}

    inline double getS(){
        return s;
    }

    inline double getD(){
        return d;
    }

    inline short getLane(){
        return calculateLane(d);
    }

    inline double getVelocityS(){
        return velocity_s;
    }

    inline void setVelocityS(double velocity_s){
        this->velocity_s = velocity_s;
    }

    inline double getAccelerationS(){
        return acceleration_s;
    }

    inline void setAccelerationS(double acceleration_s){
        this->acceleration_s = acceleration_s;
    }

    inline double getVelocityD(){
        return velocity_d;
    }

    inline void setVelocityD(double velocity_d){
        this->velocity_d = velocity_d;
    }

    inline double getAccelerationD(){
        return acceleration_d;
    }

    inline void setAccelerationD(double acceleration_d){
        this->acceleration_d = acceleration_d;
    }

    inline std::vector<double> getPredicted(double time){
        return {calculateFutureS(s + velocity_s * time + acceleration_s * pow(time, 2) / 2), d + velocity_d * time + acceleration_d * pow(time, 2) / 2};
    }

    inline double getPredictedVelocityS(double time){
        return velocity_s + acceleration_s * time;
    }

    inline double getPredictedVelocityD(double time){
        return velocity_d + acceleration_d * time;
    }

    inline double getPredictedS(double time){
        return calculateFutureS(s + velocity_s * time + .5 * acceleration_s * pow(time, 2));
    }

    inline double getPredictedD(double time){
        return d + velocity_d * time + .5 * acceleration_d * pow(time, 2);
    }

    inline void print(){
        std::cout << "[ s = " << s << ", d = " << d << ", velocity_s = " << velocity_s << ", acceleration_s = "
                  << acceleration_s << ", velocity_d = " << velocity_d << ", acceleration_d = " << acceleration_d
                  << " ]" << std::endl;
    }
};

#endif //PATH_PLANNING_VEHICLE_H
