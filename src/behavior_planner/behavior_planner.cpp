//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"



void behavior_planner::setBehavior(short lane, vehicle_behavior action){
    this->behavior[lane] = action;
}

void behavior_planner::setVelocity(short lane, float velocity){
    this->velocity[lane] = velocity;
}

void behavior_planner::setDistanceFront(short lane, float distance){
    this->distance_front[lane] = distance;
}

void behavior_planner::setDistanceBack(short lane, float distance){
    this->distance_back[lane] = distance;
}

short behavior_planner::getVelocity(short lane){
    return this->velocity[lane];
}

vehicle_behavior behavior_planner::getBehavior(short lane){
    return this->behavior[lane];
}

short behavior_planner::getDistanceFront(short lane){
    return this->distance_front[lane];
}

short behavior_planner::getDistanceBack(short lane){
    return this->distance_back[lane];
}