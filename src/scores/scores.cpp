#include <iostream>
#include "scores.h"

void scores::initialize(short lane){
    // std::cout << "scores::initialize" << std::endl;
    /*
    behavior_lock.write_lock();
    velocity_lock.write_lock();
    distance_front_lock.write_lock();
    distance_back_lock.write_lock();
     */
    // initialize the vectors for each lane
    for(short a = 0; a < num_lanes; a++){
        if(a == lane){
            behavior.push_back(KEEP_VELOCITY);
        }
        behavior.push_back(MERGE);
        velocity.push_back(max_velocity_mps);
        distance_front.push_back(spacing);
        distance_back.push_back(-spacing);
    }
    /*
    distance_back_lock.write_unlock();
    distance_front_lock.write_unlock();
    velocity_lock.write_unlock();
    behavior_lock.write_unlock();
     */
}

void scores::reset(short lane){
    // std::cout << "scores::reset" << std::endl;
    /*
    behavior_lock.write_lock();
    velocity_lock.write_lock();
    distance_front_lock.write_lock();
    distance_back_lock.write_lock();
     */
    behavior.clear();
    velocity.clear();
    distance_front.clear();
    distance_back.clear();
    /*
    distance_back_lock.write_unlock();
    distance_front_lock.write_unlock();
    velocity_lock.write_unlock();
    behavior_lock.write_unlock();
     */
    initialize(lane);
}

void scores::setFollow(short lane){
    // std::cout << "scores::setFollow" << std::endl;
    this->behavior[lane] = FOLLOW;
}

void scores::setStop(short lane){
    // std::cout << "scores::setStop" << std::endl;
    this->behavior[lane] = STOP;
}

void scores::setVelocity(short lane, double velocity){
    // std::cout << "scores::setVelocity" << std::endl;
    // velocity_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    // velocity_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    // velocity_lock.write_lock();
    // std::cout << "got write lock" << std::endl;
    this->velocity[lane] = velocity;
    // std::cout << this->velocity[lane] << std::endl;
    // velocity_lock.write_unlock();
    // std::cout << "released write lock" << std::endl;
}

void scores::setDistanceFront(short lane, double distance){
    // std::cout << "scores::setDistanceFront" << std::endl;
    // distance_front_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    // distance_front_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    // distance_front_lock.write_lock();
    // std::cout << "got write lock" << std::endl;
    this->distance_front[lane] = distance;
    // std::cout << this->distance_front[lane] << std::endl;
    // distance_front_lock.write_unlock();
    // std::cout << "released write lock" << std::endl;
}

void scores::setDistanceBack(short lane, double distance){
    // std::cout << "scores::setDistanceBack" << std::endl;
    // distance_back_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    // distance_back_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    // distance_back_lock.write_lock();
    // std::cout << "got write lock" << std::endl;
    this->distance_back[lane] = distance;
    // std::cout << this->distance_back[lane] << std::endl;
    // distance_back_lock.write_unlock();
    // std::cout << "released write lock" << std::endl;
}

double scores::getVelocity(short lane){
    // std::cout << "scores::getVelocity" << std::endl;
    // velocity_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    double out = velocity[lane];
    // std::cout << out << std::endl;
    // velocity_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    return out;
}

vehicle_behavior scores::getBehavior(short lane){
    // std::cout << "scores::getBehavior" << std::endl;
    // behavior_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    vehicle_behavior out = behavior[lane];
    // std::cout << out << std::endl;
    // behavior_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    return out;
}

double scores::getDistanceFront(short lane){
    // std::cout << "scores::getDistanceFront" << std::endl;
    // distance_front_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    double out = distance_front[lane];
    // std::cout << out << std::endl;
    // distance_front_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    return out;
}

double scores::getDistanceBack(short lane){
    // std::cout << "scores::getDistanceBack" << std::endl;
    // distance_back_lock.read_lock();
    // std::cout << "got read lock" << std::endl;
    double out = distance_back[lane];
    // std::cout << out << std::endl;
    // distance_back_lock.read_unlock();
    // std::cout << "released read lock" << std::endl;
    return out;
}

void scores::printScores(){
    /*
    velocity_lock.read_lock();
    behavior_lock.read_lock();
    distance_front_lock.read_lock();
    distance_back_lock.read_lock();
     */
    std::cout << "SCORES: " << std::endl;
    std::cout << "\tlane\t||\tbehavior\t||\tvelocity\t||\tdistance front\t||\tdistance back" << std::endl;
    for(short a = 0; a < num_lanes; a++){
        std::cout << "\t" << a << "\t||\t" << this->behavior[a] << "\t||\t" << velocity[a] << "\t||\t" << distance_front[a] << "\t||\t" << distance_back[a] << std::endl;
    }
    /*
    distance_back_lock.read_unlock();
    distance_front_lock.read_unlock();
    behavior_lock.read_unlock();
    velocity_lock.read_unlock();
     */
}