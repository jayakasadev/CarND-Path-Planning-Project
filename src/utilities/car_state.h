//
// Created by jay on 11/27/17.
//

#ifndef PATH_PLANNING_CAR_STATE_H
#define PATH_PLANNING_CAR_STATE_H

#include <vector>
#include "constants.h"
#include <iostream>

using namespace std;

enum turn{
    TURN_LEFT, STAY, TURN_RIGHT
};

enum lane_state{
    OPEN, FOLLOW, OBSTRUCTION
};

class scores{
private:
    vector<lane_state> lanes;
    vector<double> velocity;
    vector<double> distance_front;
    vector<double> distance_back;

public:
    scores(){}
    ~scores(){}

    void print(){
        cout << "MEASURES: " << endl;
        cout << " lane# || lane_state || velocity || distance" << endl;
        for(int a = 0; a < 3; a++){
            cout << "\t" << a << " || " << lanes[a] << " || " << velocity[a] << " || " << distance_front[a] << " || " << distance_back[a] << endl;
        }
    }

    inline void setup(){
        lanes.clear();
        velocity.clear();
        distance_back.clear();
        distance_front.clear();

        for(int a = 0; a < total_lanes; a++){
            lanes.push_back(OPEN);
            velocity.push_back(speed_limit);
            distance_front.push_back(spacing);
            distance_back.push_back(-spacing);
        }
    }

    inline vector<lane_state> getLaneScore(){
        return lanes;
    }
    inline vector<double> getVelocityScore(){
        return velocity;
    }
    inline vector<double> getDistanceFrontScore(){
        return distance_front;
    }
    inline vector<double> getDistanceBackScore(){
        return distance_back;
    }

    inline short getLaneScore(short lane){
        return lanes[lane];
    }
    inline double getVelocityScore(short lane){
        return velocity[lane];
    }
    inline double getDistanceFrontScore(short lane){
        return distance_front[lane];
    }
    inline double getDistanceBackScore(short lane){
        return distance_back[lane];
    }

    inline void setLaneFollow(short lane){
        lanes[lane] = FOLLOW;
    }

    inline void setLaneObstructed(short lane){
        lanes[lane] = OBSTRUCTION;
    }

    inline void setVelocityScore(short lane, double vel){
        velocity[lane] = vel;
    }
    inline void setDistanceFrontScore(short lane, double distance){
        distance_front[lane] = distance;
    }
    inline void setDistanceBackScore(short lane, double distance){
        distance_back[lane] = distance;
    }
};

#endif //PATH_PLANNING_CAR_STATE_H
