#include "traffic.h"

void traffic::update(double vx, double vy, double s, double d){
    // std::cout << "traffic::update" << std::endl;
    // print();
    if(first){
        this->velocity_s = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s
        first = false;
        this->velocity_d = 0;
    } else {
        checkOutdated(); // check if the measurements are outdated
        double temp = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s;
        std::chrono::duration<double> time_span = std::chrono::high_resolution_clock::now() - last_Seen;
        double time = time_span.count();
        this->acceleration_s = (temp - this->velocity_s) / time;
        this->velocity_s = temp;

        temp = ((d - this->d) / time);
        this->acceleration_d = (temp - this->velocity_d) / time;
        this->velocity_d = temp;
    }
    last_Seen = std::chrono::high_resolution_clock::now(); // update last seen time

    this->s = s;
    this->d = d;
}

void traffic::print() {
    std::cout << "traffic:\t";
    vehicle::print();
}