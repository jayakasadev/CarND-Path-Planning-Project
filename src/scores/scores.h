#ifndef PATH_PLANNING_SCORES_H
#define PATH_PLANNING_SCORES_H


#include <vector>
#include <iostream>

#include "../enums/vehicle_behavior.h"
#include "../constants/road_constants.h"
#include "../utilities/read_write_lock.h"

class scores {
private:
    read_write_lock lock;

    void initialize(double s , short lane);

public:
    scores(){}

    ~scores(){}

    void reset(double s, short lane);

    void printScores();
};


#endif //PATH_PLANNING_SCORES_H
