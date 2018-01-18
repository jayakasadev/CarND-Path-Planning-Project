#include "detections.h"

void detections::add(traffic &vehicle){
    lock.write_lock();
    detected.push_back(&vehicle);
    lock.write_unlock();
}

traffic * detections::iterateDetections(){
    lock.read_lock();
    if(itr >= detected.size()){
        lock.read_unlock();
        return nullptr;
    }
    traffic * out = detected[itr++];
    lock.read_unlock();
    return out;
}

void detections::reset(){
    lock.write_lock();
    detected.clear();
    itr = 0;
    lock.write_unlock();
}

void detections::print(){
    std::cout << "Detections:" << std::endl;
    lock.read_lock();
    for(short a = 0; a < detected.size(); a++){
        detected[a]->print();
    }
    lock.read_unlock();
}