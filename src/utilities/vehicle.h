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

class vehicle{
public:
    vehicle(){};
    ~vehicle(){};
protected:
    double x;
    double y;
    double yaw;
    double d;
};

class other_vehicle : public vehicle{
private:
    double vx;
    double vy;
    double ax;
    double ay;
    bool first;

    high_resolution_clock::time_point last_Seen;

public:
    other_vehicle(){
        first = true;
        yaw = 0;
    };
    ~other_vehicle(){};

    inline void updateVehicle(double vx_val, double vy_val, double x, double y, double d){
        if(!first){
            this->ax = (vx_val - this->vx) / time_interval; // calculate acceleration
            this->ay = (vy_val - this->vy) / time_interval; // calculate acceleration
            // get yaw before updating position
            yaw = mapData.getYaw(y, this->y, x, this->x);
        }
        else{
            this->ax = this->ay = 0;
            first = false;
        }

        this->vx = vx_val; // set velocity
        this->vy = vy_val; // set velocity

        this->x = x;
        this->y = y;
        this->d = d;

        last_Seen = high_resolution_clock::now();
    }

    inline short getLane(){
        return d / 4;
    }

    inline void predict(double &s, double &d, double &velocity){
        // predict the position
        double px = x + vx * predict_window + ax * pow(predict_window, 2);
        double py = y + vy * predict_window + ay * pow(predict_window, 2);
        vector<double> vals = mapData.getFrenet(px, py, yaw);
        s = vals[0];
        d = vals[1];

        double pvx = vx + ax * predict_window;
        double pvy = vy + ay * predict_window;
        velocity = sqrt(pow(pvx, 2) + pow(pvy, 2)); // predict the velocity
    }

    inline long updateTime(){
        return (high_resolution_clock::now() - last_Seen).count();
    }
};

class driver : public vehicle{
private:
    double velocity;
    double acceleration;
    double s;
    bool first;

    turn turn_type;
public:

    driver():first(true){}
    ~driver(){}

    inline void updateVehicle(double v_val, double x, double y, double s, double d){
        if(!first){
            this->acceleration = (v_val - this->velocity) / time_interval; // calculate acceleration
        } else {
            this->acceleration = 0;
            first = false;
        }

        this->velocity = v_val; // set velocity

        yaw = mapData.getYaw(y, this->y, x, this->x);

        this->x = x;
        this->y = y;
        this->d = d;

        this->s = s;
    }

    inline void setTurnType(turn type){
        this->turn_type = type;
    }

    inline short getLane(){
        return d / 4;
    }

    inline turn getTurnType(){
        return turn_type;
    }

    inline double getS(){
        return s;
    }

    inline double predict(){
        return s + velocity * time_interval + acceleration * pow(time_interval, 2);
    }
};


#endif //PATH_PLANNING_VEHICLE_H
