//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <math.h>
#include <iostream>
#include <chrono>

#include "../constants/constants.h"
#include "utilities.h"
#include "read_write_lock.h"

using namespace std::chrono;

struct vehicle{
protected:
    float x;
    float y;

    float s;
    float d;

    float yaw;

    float velocity; // in m/s

    float acceleration; // in m/s^2

    bool first;

public:
    vehicle(){}
    ~vehicle(){}

    inline float getS(){
        return s;
    }

    inline float getD(){
        return d;
    }

    inline float getYaw(){
        return yaw;
    }

    inline float getVelocity(){
        return velocity;
    }

    inline int getLane(){
        return calculateLane(d);
    }

    inline float getAcceleration(){
        return acceleration;
    }

    inline void print(){
        std::cout << "[ s = " << s << ", d = " << d << ", yaw = " << yaw << ", velocity = " << velocity << ", acceleration = " << acceleration << " ]" << std::endl;
    }
};

class driver : public vehicle {
public:
    // constructor for driver
    driver(){
        acceleration = 0;
    }

    ~driver(){}

    void update(float x, float y, float s, float d, float yaw, float speed){
        if(first){
            this->velocity = speed * mph_to_mps; // convert mph to m/s
            first = false;
        } else {
            // calculate acceleration
            float temp = speed * mph_to_mps;
            this->acceleration = (temp - this->velocity) / refresh_rate;
            this->velocity = temp;
        }

        this->x = x;
        this->y = y;

        this->s = s;
        this->d = d;

        this->yaw = deg2rad(yaw);
    }

    inline void print(){
        std::cout << "driver:\t";
        vehicle::print();
    }
};

class others : public vehicle {
private:
    float vx;
    float vy;
    float px;
    float py;
    std::chrono::high_resolution_clock::time_point last_Seen;
    read_write_lock lock;

    inline void checkOutdated(){
        lock.read_lock();
        long time = (high_resolution_clock::now() - last_Seen).count();
        if(time > search_field_timelimit){ // last
            first = true;
            acceleration = 0;
            yaw = 0;
        }
        lock.read_unlock();
    }

public:
    // constructor for sensor fusion
    others(){
        acceleration = 0;
        yaw = 0;
        last_Seen = high_resolution_clock::now(); // update last seen time
    }

    ~others(){}

    void update(float x, float y, float vx, float vy, float s, float d){
        checkOutdated();
        lock.write_lock();
        if(first){
            this->velocity = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s
            first = false;

            px = x + vx * time_period;
            py = y + vy * time_period;
        } else {
            float temp = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s;
            this->acceleration = (temp - this->velocity) / refresh_rate;
            this->velocity = temp;

            // predictions
            px = x + vx * time_period + ((vx - this->vx) / refresh_rate) * pow(time_period, 2); // predict x location 1 sec in future
            py = y + vy * time_period + ((vy - this->vy) / refresh_rate) * pow(time_period, 2); // predict y location 1 sec in future

            this->yaw = calculateYaw(y, py, x, py);
        }
        last_Seen = high_resolution_clock::now(); // update last seen time

        this->vx = vx;
        this->vy = vy;

        this->x = x;
        this->y = y;

        this->s = s;
        this->d = d;

        lock.write_unlock();
    }

    inline void print(){
        lock.read_lock();
        std::cout << "others:\t";
        vehicle::print();
        lock.read_unlock();
    }

    inline std::vector<float> getPredicted(){
        return {px, py};
    }
};

#endif //PATH_PLANNING_VEHICLE_H
