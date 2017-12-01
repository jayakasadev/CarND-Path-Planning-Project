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

    inline double getD(){
        return d;
    }
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

    inline long updateTime(){
        return (high_resolution_clock::now() - last_Seen).count();
    }

public:
    other_vehicle(){
        first = true;
        yaw = 0;
    };
    ~other_vehicle(){};

    inline void updateVehicle(double vx_val, double vy_val, double x, double y, double d, map_data &mapData){
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

    inline void predict(double &s, double &d, double &velocity, map_data &mapData){
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

    inline bool outDated(){
        if(updateTime() >= time_diff){
            first = true;
            return true;
        }
        return false;
    }
};

class driver : public vehicle{
private:
    double velocity_d;
    double velocity_s;
    double acceleration_s;
    double acceleration_d;
    double s;
    bool first;
    short desired_lane;

    turn turn_type;
public:

    driver():first(true), turn_type(STAY),desired_lane(1){}
    ~driver(){}

    inline void updateVehicle(double v_val, double x, double y, double s, double d, map_data &mapData){
        if(!first){
            this->acceleration_s = (v_val - this->velocity_s) / time_interval; // calculate acceleration

            double vel_d = (d - this->d) / time_interval;
            this->acceleration_d = (vel_d - velocity_d) / time_interval;
            this->velocity_d = vel_d;
        } else {
            this->acceleration_s = acceleration_d = 0;
            velocity_d = 0;
            first = false;
        }

        this->velocity_s = v_val; // set velocity

        yaw = mapData.getYaw(y, this->y, x, this->x);

        this->x = x;
        this->y = y;

        this->d = d;

        this->s = s;
    }

    inline void setTurnType(turn type){
        this->turn_type = type;
    }

    inline void setDesiredLane(short lane){
        this->desired_lane = lane;
    }

    inline short getDesiredLane(){
        return desired_lane;
    }

    inline short getLane(){
        return d / 4;
    }

    inline turn getTurnType(){
        return turn_type;
    }

    inline double & getS(){
        return s;
    }

    inline double getVelocityS(){
        return velocity_s;
    }

    inline double getVelocityD(){
        return velocity_d;
    }

    inline double getAccelerationS(){
        return acceleration_s;
    }

    inline double getAccelerationD(){
        return acceleration_d;
    }

    inline double predictS(){
        return s + velocity_s * time_interval + acceleration_s * pow(time_interval, 2);
    }

    inline double predictD(){
        return d + velocity_d * time_interval + acceleration_d * pow(time_interval, 2);
    }
};


#endif //PATH_PLANNING_VEHICLE_H
