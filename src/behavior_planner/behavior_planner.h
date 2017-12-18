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
    double score_s;
    double score_d;
    VectorXd s;
    VectorXd d;

    trajectory_option(){
        score_s = numeric_limits<float>::max();
        score_d = numeric_limits<float>::max();
    }
    ~trajectory_option(){}
};

class behavior_planner {
private:
    driver *car;
    scores *values;
    vector<MatrixXd> times;

    inline void getSfVals(double &sf, double &sf_dot, short lane){
        // values->printScores();
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

    inline double squareJerk(VectorXd &x, double constant){
        VectorXd c(3);
        c << 6 * constant, 24 * pow(constant, 2), 60 * pow(constant, 3);
        return pow(x.transpose() * c, 2);
    }

    // cost functions
    inline double costS(short lane, short index, double diff, VectorXd &calculated){
        double time = time_period + refresh_rate * index;
        cout << "behavior_planner::costS: [ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time) << ", kt = " << k_t << ", time = " << time << ", ks = " << k_s << ", diff^2 = " << pow(diff, 2) << " ]" << endl;
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time) + k_t * time + k_s * pow(diff, 2);
    }

    inline double costD(short lane, short index, double diff, VectorXd &calculated){
        double time = time_period + refresh_rate * index;
        cout << "behavior_planner::costD: [ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time) << ", kt = " << k_t << ", time = " << time << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2) << " ]" << endl;
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time) + k_t * time + k_d * pow(diff, 2);
    }

    inline double costSport(short &lane, VectorXd &calculated){
        // println("behavior_planner::costSport");
        return  0;
    }

    inline double costEconomy(short &lane, VectorXd &calculated){
        // println("behavior_planner::costEconomy");
        return  0;
    }

    // behavior based on velocity

    trajectory_option* highwayPlanning(short lane);

    trajectory_option* highwayPlanningAtD(short lane, double distance);

    trajectory_option cityPlanning(short lane);

    inline void sharedCalc(short index, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, VectorXd &c){
        // cout << "sharedCalc: [ time = " << time << ", x = " << x << ", x_dot = " << x_dot << ", x_dot_dot = " << x_dot_dot << ", xf = " << xf << ", xf_dot = " << xf_dot << ", xf_dot_dot = " << xf_dot_dot << endl;
        // cout << "\nA^-1:\n" << Ai << endl;

        VectorXd b = VectorXd::Zero(3);

        double time = time_period + refresh_rate * index;

        b << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
                (xf_dot - (x_dot + x_dot_dot * time)),
                (xf_dot_dot - x_dot_dot);

        // cout << "b = " << b.transpose() << endl;

        c = times.at(index) * b;
        // cout << c.transpose() << endl;
    }



public:
    behavior_planner(driver &car, scores &values){
        this->car = &car;
        this->values = &values;
    }

    ~behavior_planner(){
        for(short a = 0; a <= num_points; a++){
            double time = time_period + refresh_rate * a;
            MatrixXd A = MatrixXd::Zero(3, 3);
            A << pow(time, 3), pow(time, 4), pow(time, 5),
                    3 * pow(time, 2), 4 *pow(time, 3), 5 * pow(time, 4),
                    6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

            // cout << "A" << endl;
            // cout << A << endl;
            times.push_back(A.inverse());
        }
        // cout << times.size() << endl;
    }

    vector<VectorXd> bestOption();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
