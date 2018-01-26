//
// Created by jay on 12/9/17.
//

#include "driver.h"

void driver::initialize(double s, double d, double speed){
    this->s = s;
    this->d = d;

    this->velocity_s = speed ; // convert mph to m/s
    this->velocity_d = 0;

    this->acceleration_s = 0;
    this->acceleration_d = 0;
}

void driver::update(double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot){
    this->s = s;
    this->velocity_s = s_dot;
    this->acceleration_s = s_dot_dot;

    this->d = d;
    this->velocity_d = d_dot;
    this->acceleration_d = d_dot_dot;
}