//
// Created by jay on 12/9/17.
//

#include "traffic.h"

void traffic::update(double x, double y, double vx, double vy, double s, double d){
    // std::cout << "traffic::update" << std::endl;
    // print();
    if(first){
        this->velocity_s = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s
        // std::cout << " velocity_s : " << this->velocity_s << std::endl;
        first = false;
        this->velocity_d = 0;
        // px = x + vx * time_period;
        // py = y + vy * time_period;
    } else {
        checkOutdated(); // check if the measurements are outdated
        double temp = sqrt(pow(vx, 2) + pow(vy, 2)); // vx and vy are in m/s;
        // duration<double> time_span = high_resolution_clock::now() - last_Seen;
        // std::cout << "temp : " << temp << " velocity_s : " << this->velocity_s << std::endl;
        // this->acceleration_s = (temp - this->velocity_s) / time_span.count();
        this->velocity_s = temp;

        // std::cout << "acceleration_s " << this->acceleration_s << std::endl;

        temp = ((d - this->d) / refresh_rate);
        // std::cout << "temp : " << temp << " velocity_d : " << velocity_d << std::endl;
        // this->acceleration_d = (temp - this->velocity_d) / time_span.count();
        // std::cout << "acceleration_d " << this->acceleration_d << std::endl;
        this->velocity_d = temp;
        // predictions
        // px = x + vx * time_period + ((vx - this->vx) / refresh_rate) * pow(time_period, 2); // predict x location 1 sec in future
        // py = y + vy * time_period + ((vy - this->vy) / refresh_rate) * pow(time_period, 2); // predict y location 1 sec in future

        // this->yaw = calculateYaw(y, py, x, py);
    }
    last_Seen = high_resolution_clock::now(); // update last seen time

    // ps = calculateFutureS(s + this->velocity_s * time_period);
    // pd = d + this->velocity_d * time_period;

    this->vx = vx;
    this->vy = vy;

    this->x = x;
    this->y = y;

    this->s = s;
    this->d = d;
}

void traffic::print() {
    std::cout << "traffic:\t";
    vehicle::print();
}