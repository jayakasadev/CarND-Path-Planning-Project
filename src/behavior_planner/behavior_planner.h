//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <future>
#include <vector>

#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/constants.h"
#include "../enums/vehicle_behavior.h"
#include "../scores/scores.h"
#include "../vehicle/driver.h"

using namespace Eigen;
using namespace std;

struct trajectory_option{
    float score_s;
    float score_d;
    VectorXd s;
    VectorXd d;

    trajectory_option(){
        score_s = score_d = numeric_limits<float>::max();
    }
    ~trajectory_option(){}
};

class behavior_planner {
private:
    driver *car;
    scores *values;

    inline void getSfVals(double &sf, double &sf_dot, short lane){
        values->printScores();
        // car->print();
        // cout << "behavior: " << values->getBehavior(lane);
        vehicle_behavior behavior = values->getBehavior(lane);
        if(behavior == vehicle_behavior::STOP) {
            // cout << "STOP " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() + values->getDistanceFront(lane);
        } else if(behavior == vehicle_behavior::FOLLOW) {
            // cout << "FOLLOW " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() + values->getDistanceFront(lane) + values->getVelocity(lane) * time_period;
            sf_dot = values->getVelocity(lane);
        } else if(behavior == vehicle_behavior::MERGE){
            // cout << "MERGE " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() + values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        } else{
            // cout << "KEEP VELOCITY " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() +  + values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        }
        // cout << " sf: " << sf << " sf_dot: " << sf_dot << endl;
    }

    inline double calcYVal(VectorXd x, double constant){
        VectorXd c(6);
        c << constant, pow(constant, 2), pow(constant, 3), pow(constant, 4), pow(constant, 5), pow(constant, 6);
        return x.transpose() * c;
    }

    // cost functions

    double cost(short lane, VectorXd &calculated);

    double costSport(short lane, VectorXd &calculated);

    double costEconomy(short lane, VectorXd &calculated);

    // behavior based on velocity

    trajectory_option highwayPlanning(short lane);

    trajectory_option cityPlanning(short lane);

    inline void sharedCalc(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, VectorXd &c){
        MatrixXd A(3, 3);
        A << pow(time, 3), pow(time, 4), pow(time, 5),
                3 * pow(time, 2), 4 *pow(time, 3), 5 * pow(time, 4),
                6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

        A = A.inverse();
        // cout << "A^-1: " << A << endl;

        VectorXd b(3);

        b << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
                (xf_dot - (x_dot + x_dot_dot * time)),
                (xf_dot_dot - x_dot_dot);

        c = A.inverse() * b;
    }



public:
    behavior_planner(driver &car, scores &values){
        this->car = &car;
        this->values = &values;
    }

    ~behavior_planner(){}

    vector<VectorXd> bestOption();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
