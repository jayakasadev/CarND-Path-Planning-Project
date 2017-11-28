//
// Created by jay on 11/27/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <cmath>
#include <chrono>
#include "car_state.h"
#include "constants.h"
#include "utilities.h"

using namespace std::chrono;

struct vehicle{
protected:

    double x;
    double y;

    double yaw;
    short lane;
};

class other_vehicle : public vehicle{
private:
    double vx;
    double vy;
    double ax;
    double ay;

    high_resolution_clock::time_point last_Seen;

public:
    other_vehicle(double vx_val, double vy_val, double x, double y, double d_val){
        ax = ay = 0;
        yaw = 0;
        this->x = x;
        this->y = y;
        this->vx = vx_val;
        this->vy = vy_val;

        lane = (d_val-2) / 4;

        last_Seen = high_resolution_clock::now();
    }
    ~other_vehicle(){};

    inline void updateVehicle(double vx_val, double vy_val, double x, double y, double d){
        this->ax = (vx_val - this->vx) / time_interval; // calculate acceleration
        this->ay = (vy_val - this->vy) / time_interval; // calculate acceleration

        this->vx = vx_val; // set velocity
        this->vy = vy_val; // set velocity

        this->x = x;
        this->y = y;
        lane = (d-2) / 4;

        yaw = getYaw(y, this->y, x, this->x);
    }

    inline short getLane(){
        return lane;
    }

    inline void predict(double &s, double &d, double &velocity, int prev_size){
        // predict the position
        double px = x + vx * (time_interval * prev_size) + ax * pow(time_interval* prev_size, 2);
        double py = y + vy * time_interval + ay * pow(time_interval, 2);
        vector<double> vals = getFrenet(px, py, yaw);
        s = vals[0];
        d = vals[1];

        double pvx = vx + ax * time_interval * prev_size;
        double pvy = vy + ay * time_interval * prev_size;
        velocity = sqrt(pow(pvx, 2) + pow(pvy, 2)); // predict the velocity
    }

    inline long updateTime(){
        high_resolution_clock::time_point curr = high_resolution_clock::now();
        long diff = (curr - last_Seen).count();
        last_Seen = curr;
        return diff;
    }
};

class driver : public vehicle{
private:
    double velocity;
    double acceleration;
    double s;

    turn turn_type;
public:

    driver(double v_val, double x, double y, double s, double d_val){
        acceleration = 0;
        turn_type = UNKNOWN;
        yaw = 0;

        this->x = x;
        this->y = y;
        this->velocity = v_val;

        lane = (d_val-2) / 4;

        this->s = s;
    }

    ~driver(){}

    inline void updateVehicle(double v_val, double x, double y, double s, double d){
        this->acceleration = (v_val - this->velocity) / time_interval; // calculate acceleration

        this->velocity = v_val; // set velocity

        this->x = x;
        this->y = y;
        lane = (d-2) / 4;

        this->s = s;

        yaw = getYaw(y, this->y, x, this->x);
    }

    inline void setTurnType(turn type){
        this->turn_type = type;
    }

    inline short getLane(){
        return lane;
    }

    inline turn getTurnType(){
        return turn_type;
    }

    inline double getS(){
        return s;
    }
};


#endif //PATH_PLANNING_VEHICLE_H
