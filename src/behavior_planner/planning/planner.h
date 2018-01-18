#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "../../vehicle/driver.h"
#include "../../detections/detections.h"
#include "../trajectory_option/trajectory_option.h"
#include "../../trajectory/trajectory.h"

using namespace std;

class planner{

public:

    planner(driver &car, detections &detected, vector<trajectory_option *> &calculators, short lane){
        cout << "planner constructor" << endl;
        // cout << "city_planner" << endl;
        this->car = &car;
        this->calculators = calculators;
        this->detected = &detected;
        this->lane = lane;

        for(short a = 0; a < (num_generated_s_points * (num_points + 1)); a++){
            // text all possible time T for each generated point before moving to the next
            trajectory * s_traj = new trajectory(trajectory_type::S, time_period + (a % (num_points + 1)) * refresh_rate);
            s_trajectories.push_back(s_traj);
        }

        for(short a = 0; a < (num_points + 1); a++){
            // text all possible time T for each generated point before moving to the next
            trajectory * traj_d = new trajectory(trajectory_type::D, time_period + a * refresh_rate);
            d_trajectories.push_back(traj_d);
        }
    }

    ~planner(){
        cout << "planner destructor" << endl;
        for(short a = 0; a < s_trajectories.size(); a++){
            delete s_trajectories[a];
        }
        s_trajectories.clear();

        for(short a = 0; a <  d_trajectories.size(); a++){
            delete d_trajectories[a];
        }
        d_trajectories.clear();
    }

protected:
    vector<trajectory_option *> calculators;
    vector<trajectory *> s_trajectories;
    vector<trajectory *> d_trajectories;
    vector<double> generated;
    driver * car;
    detections * detected;
    short lane;

    /**
     * Method to generate random points
     */
    inline void generatePoints(){
        generated.clear();
        for(short a = 0; a < num_generated_s_points; a++){
            double random = (double)rand() / RAND_MAX;
            generated.push_back(s_search_start_point + random * (spacing - s_search_start_point));
        }
    }

    /**
     * Method to print randomly generated points
     */
    inline void printGenerated(){
        for(short a = 0; a < generated.size(); a++){
            cout << "generated: " << generated[a] << endl;
        }
    }

};

#endif //PATH_PLANNING_PLANNER_H
