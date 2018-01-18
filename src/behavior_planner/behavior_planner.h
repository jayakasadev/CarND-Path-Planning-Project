//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <future>
#include <vector>

#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/road_constants.h"
#include "../constants/behavior_constants.h"
#include "../vehicle/driver.h"
#include "planning/city_planner.h"
#include "planning/highway_planner.h"
#include "../detections/detections.h"
#include "../trajectory/trajectory.h"
#include "trajectory_option/trajectory_option.h"

using namespace Eigen;
using namespace std;

class behavior_planner {
private:
    vector<highway_planner *> highwayPlanner;
    vector<city_planner *> cityPlanner;
    vector<trajectory_option *> calculators;
    driver * car;

public:
    behavior_planner(driver &car, detections &detected){
        cout << "behavior_planner constructor" << endl;
        this->car = &car;

        for(short a = 0; a <= num_points; a++){
            trajectory_option * opt = new trajectory_option(time_period + refresh_rate * a);
            calculators.push_back(opt);
        }

        for(short a = 0; a < num_lanes; a++){
            highway_planner * hp = new highway_planner(car, detected, calculators, a);
            highwayPlanner.push_back(hp);
            city_planner * cp = new city_planner(car, detected, calculators, a);
            cityPlanner.push_back(cp);
        }
        // cout << "size highway: " << highwayPlanner.size() << "\tsize city: " << cityPlanner.size() << endl;
        // highwayPlanner[0].calculate();
        // cityPlanner[0].calculate();
    }

    ~behavior_planner(){
        cout << "behavior_planner destructor" << endl;
        /*
        for(short a = 0; a < calculators.size(); a++){
            delete calculators[a];
        }
        calculators.clear();
         */
    }

    vector<trajectory> plan();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
