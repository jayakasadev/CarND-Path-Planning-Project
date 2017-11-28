//
// Created by jay on 11/27/17.
//

#ifndef PATH_PLANNING_CAR_STATE_H
#define PATH_PLANNING_CAR_STATE_H

enum turn{
    TURN_LEFT, STAY, TURN_RIGHT, UNKNOWN
};

enum lane_state{
    OPEN, OBSTRUCTION, FOLLOW
};

#endif //PATH_PLANNING_CAR_STATE_H
