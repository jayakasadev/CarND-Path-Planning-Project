#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

#include <unordered_map>

#include "../utilities/json.hpp"
#include "../vehicle/driver.h"
#include "../vehicle/traffic.h"
#include "../tunable_params/sensor_fusion_tunable.h"
#include "../utilities/pools/pointer_pool.h"

class sensorfusion {
private:
    std::unordered_map<short, std::shared_ptr<traffic>> hashmap;
    std::shared_ptr<driver> car;
    std::shared_ptr<pointer_pool<traffic>> detected;

    inline float getSearchRadius() {
        return search_radius + growth_rate * car->getVelocityS();
    }

public:
    sensorfusion(std::shared_ptr<driver> car, std::shared_ptr<pointer_pool<traffic>> detected){
        // std::cout << "sensorfusion constructor" << std::endl;
        this->car = car;
        this->detected = detected;
        // car->print();
    }

    ~sensorfusion(){
        std::cout << "sensorfusion destructor" << std::endl;
    }

    void predict(nlohmann::basic_json<> &sensor_fusion, double size);
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
