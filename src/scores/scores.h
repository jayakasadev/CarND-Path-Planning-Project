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

    read_write_lock lock;
    // read_write_lock behavior_lock;
    // read_write_lock velocity_lock;
    // read_write_lock distance_front_lock;
    // read_write_lock distance_back_lock;

    void initialize(double s , short lane);

public:
    scores(){}

    ~scores(){}

    void reset(double s, short lane);

    void setFollow(short lane);

    void setStop(short lane);

    void setVelocity(short lane, double velocity);

    void setDistanceFront(short lane, double distance);

    void setDistanceBack(short lane, double distance);

    double getVelocity(short lane);

    vehicle_behavior getBehavior(short lane);

    double getDistanceFront(short lane);

    double getDistanceBack(short lane);

    void printScores();
};


#endif //PATH_PLANNING_SCORES_H
