#include "scores.h"

void scores::initialize(double s, short lane){

}

void scores::reset(double s, short lane){

}

void scores::setFollow(short lane){

}

void scores::setStop(short lane){

}

void scores::setVelocity(short lane, double velocity){

}

void scores::setDistanceFront(short lane, double distance){

}

void scores::setDistanceBack(short lane, double distance){

}

double scores::getVelocity(short lane){
    return 0;
}

vehicle_behavior scores::getBehavior(short lane){

}

double scores::getDistanceFront(short lane){

}

double scores::getDistanceBack(short lane){

}

void scores::printScores(){
    /*
    velocity_lock.read_lock();
    behavior_lock.read_lock();
    distance_front_lock.read_lock();
    distance_back_lock.read_lock();
     */
    lock.read_lock();
    std::cout << "SCORES: " << std::endl;
    std::cout << "\tlane\t||\tbehavior\t||\tvelocity\t||\tdistance front\t||\tdistance back" << std::endl;
    for(short a = num_lanes - 1; a >= 0; a--){
        // std::cout << "\t" << a << "\t||\t" << this->behavior[a] << "\t||\t" << this->velocity[a] << "\t||\t" << this->distance_front[a] << "\t||\t" << this->distance_back[a] << std::endl;
    }
    lock.read_unlock();
    /*
    distance_back_lock.read_unlock();
    distance_front_lock.read_unlock();
    behavior_lock.read_unlock();
    velocity_lock.read_unlock();
     */
}