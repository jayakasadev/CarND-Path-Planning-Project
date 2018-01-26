#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <chrono>
#include "vehicle.h"
#include "../tunable_params/sensor_fusion_tunable.h"

class traffic : public vehicle {
private:
    std::chrono::high_resolution_clock::time_point last_Seen;
    bool first;

    inline void checkOutdated() {
        // std::cout << "checkOutdated" << std::endl;
        std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - last_Seen;
        short time = time_span.count();
        // std::cout << "time = " << time << std::endl;
        if (time > time_limit) { // last
            velocity_d = 0;
            velocity_s = 0;
            s = 0;
            d = 0;
            acceleration_s = 0;
            acceleration_d = 0;
        }
    }

public:
    // constructor for sensor fusion
    traffic() {
        last_Seen = std::chrono::high_resolution_clock::now(); // update last seen time
        first = true;
    }

    traffic(const traffic &obj) {
        std::cout << "traffic copy constructor" << std::endl;
        this->last_Seen = obj.last_Seen; // update last seen time
        this->first = obj.first;

        this->velocity_d = obj.velocity_d;
        this->velocity_s = obj.velocity_s;

        this->s = obj.s;
        this->d = obj.d;

        this->acceleration_s = obj.acceleration_s;
        this->acceleration_d = obj.acceleration_d;
    }

    ~traffic() {
        std::cout << "traffic destructor" << std::endl;
    }

    void update(double vx, double vy, double s, double d);

    virtual std::ostream& print(std::ostream& os) const {
        return os << "traffic\t";
    }
};


#endif //PATH_PLANNING_TRAFFIC_H
