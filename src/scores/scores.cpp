#include <iostream>
#include "scores.h"

void scores::initialize(){
    // initialize the vectors for each lane
    for(short a = 0; a < num_lanes; a++){
        behavior.push_back(KEEP_VELOCITY);
        velocity.push_back(max_velocity_mps);
        distance_front.push_back(spacing);
        distance_back.push_back(spacing);
    }
}

void scores::reset(){
    behavior.clear();
    velocity.clear();
    distance_front.clear();
    distance_back.clear();
    initialize();
}

void scores::setBehavior(short lane, vehicle_behavior action){
    if(this->behavior[lane] != action){
        behavior_lock.write_lock();
        this->behavior[lane] = action;
        behavior_lock.write_unlock();
    }
}

void scores::setVelocity(short lane, float velocity){
    if(this->velocity[lane] > velocity){
        velocity_lock.write_lock();
        this->velocity[lane] = velocity;
        velocity_lock.write_unlock();
    }
}

void scores::setDistanceFront(short lane, float distance){
    if(this->distance_front[lane] > distance){
        distance_front_lock.write_lock();
        this->distance_front[lane] = distance;
        distance_front_lock.write_unlock();
    }
}

void scores::setDistanceBack(short lane, float distance){
    if(this->distance_back[lane] > distance){
        distance_back_lock.write_lock();
        this->distance_back[lane] = distance;
        distance_back_lock.write_unlock();
    }
}

short scores::getVelocity(short lane){
    velocity_lock.read_lock();
    double out = velocity[lane];
    velocity_lock.read_unlock();
    return out;
}

vehicle_behavior scores::getBehavior(short lane){
    behavior_lock.read_lock();
    vehicle_behavior out = behavior[lane];
    behavior_lock.read_unlock();
    return out;
}

short scores::getDistanceFront(short lane){
    distance_front_lock.read_lock();
    double out = distance_front[lane];
    distance_front_lock.read_unlock();
    return out;
}

short scores::getDistanceBack(short lane){
    distance_back_lock.read_lock();
    double out = distance_back[lane];
    distance_back_lock.read_unlock();
    return out;
}

void scores::printScores(){
    velocity_lock.read_lock();
    behavior_lock.read_lock();
    distance_front_lock.read_lock();
    distance_back_lock.read_lock();

    std::cout << "SCORES: " << std::endl;
    std::cout << "\tlane\t||\tbehavior\t||\tvelocity\t||\tdistance front\t||\tdistance back" << std::endl;
    for(short a = 0; a < num_lanes; a++){
        std::cout << "\t" << a << "\t||\t" << this->behavior[a] << "\t||\t" << velocity[a] << "\t||\t" << distance_front[a] << "\t||\t" << distance_back[a] << std::endl;
    }

    velocity_lock.read_unlock();
    behavior_lock.read_unlock();
    distance_front_lock.read_unlock();
    distance_back_lock.read_unlock();


}