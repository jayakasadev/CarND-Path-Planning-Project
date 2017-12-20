//
// Created by jay on 12/9/17.
//

#include "driver.h"

void driver::update(double x, double y, double s, double d, double yaw, double speed){

    this->velocity_s = speed * mph_to_mps; // convert mph to m/s
    /*
    if(first){
        this->velocity_s = speed * mph_to_mps; // convert mph to m/s
        // this->velocity_d = 0;
        // first = false;
    } else {
        // calculate acceleration
        double temp = speed * mph_to_mps;
        // this->acceleration_s = (temp - this->velocity_s) / refresh_rate;
        this->velocity_s = temp;

        // temp = (d - this->d) / refresh_rate;
        // this->acceleration_d = (temp - this->velocity_d) / refresh_rate;
        // this->velocity_d = temp;
    }
    */

    this->x = x;
    this->y = y;

    this->s = s;
    this->d = d;

    this->yaw = deg2rad(yaw);

    ps = calculateFutureS(s + this->velocity_s * time_period + .5 * this->acceleration_s * pow(time_period, 2));
    pd = d + this->velocity_d * time_period + .5 * this->acceleration_d * pow(time_period, 2);
}

void driver::print(){
    std::cout << "driver:\t";
    vehicle::print();
}