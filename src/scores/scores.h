//
// Created by jay on 12/8/17.
//

#ifndef PATH_PLANNING_SCORES_H
#define PATH_PLANNING_SCORES_H


#include <vector>
#include "../enums/vehicle_behavior.h"
#include "../constants/constants.h"
#include "../utilities/read_write_lock.h"

class scores {
private:
    std::vector<vehicle_behavior> behavior;
    std::vector<double> velocity;
    std::vector<double> distance_front;
    std::vector<double> distance_back;

    read_write_lock behavior_lock;
    read_write_lock velocity_lock;
    read_write_lock distance_front_lock;
    read_write_lock distance_back_lock;

public:
    scores(short lane){
        initialize(lane);
    }
    ~scores(){}

    void reset(short lane);

    void initialize(short lane);

    void setBehavior(short lane, vehicle_behavior action);

    void setVelocity(short lane, double velocity);

    void setDistanceFront(short lane, double distance);

    void setDistanceBack(short lane, double distance);

    short getVelocity(short lane);

    vehicle_behavior getBehavior(short lane);

    short getDistanceFront(short lane);

    short getDistanceBack(short lane);

    void printScores();
};


#endif //PATH_PLANNING_SCORES_H
