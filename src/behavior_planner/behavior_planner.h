//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <vector>
#include "../constants/constants.h"

enum vehicle_behavior{
    KEEP_VELOCITY, FOLLOW, MERGE, STOP
};

class behavior_planner {
private:
    std::vector<vehicle_behavior> behavior;
    std::vector<double> velocity;
    std::vector<float> distance_front;
    std::vector<float> distance_back;
public:
    behavior_planner(){
        // initialize the vectors for each lane
        for(short a = 0; a < num_lanes; a++){
            behavior.push_back(KEEP_VELOCITY);
            velocity.push_back(max_velocity_mps);
            distance_front.push_back(spacing);
            distance_back.push_back(spacing);
        }
    }
    ~behavior_planner(){
        delete behavior;
        delete velocity;
        delete distance_front;
        delete distance_back;
    }

    void setBehavior(short lane, vehicle_behavior action);

    void setVelocity(short lane, float velocity);

    void setDistanceFront(short lane, float distance);

    void setDistanceBack(short lane, float distance);

    short getVelocity(short lane);

    vehicle_behavior getBehavior(short lane);

    short getDistanceFront(short lane);

    short getDistanceBack(short lane);
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
