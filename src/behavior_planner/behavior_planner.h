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
        this->score_s = numeric_limits<float>::max();
        this->score_d = numeric_limits<float>::max();
        this->s = VectorXd::Zero(6);
        this->d = VectorXd::Zero(6);
    }
    ~trajectory_option(){}
};

class behavior_planner {
private:
    driver *car;
    scores *values;

    // weights for the cost function
    // max value is 7500 for a bad score
    // best score is 0

    const float k_j = 1;

    const float k_d = 39.0625;

    const float k_t = 156.25;

    const float k_s = -4;

    const float k_s_bias = 2500;

    const float k_v = 25;

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

    inline double positionF(VectorXd &x, double constant, double s, double s_dot, double s_dot_dot){
        VectorXd c(3);
        c << pow(constant, 3) / 6, pow(constant, 4) / 24, pow(constant, 5) / 120;
        return x.transpose() * c + .5 * s_dot_dot * pow(constant, 2) + s_dot * constant + s;
    }

    inline double velocityF(VectorXd &x, double s_dot, double s_dot_dot){
        /*
        double sum = 0;
        for(short a = 0; a <= num_points; a++){
            VectorXd c(3);
            double constant = refresh_rate * a;
            c << pow(constant, 2) / 2, pow(constant, 3) / 6, 5 * pow(constant, 4) / 24;
            sum += x.transpose() * c + s_dot_dot * constant + s_dot;
        }
        return (sum / (num_points + 1)); // average velocity
         */
        VectorXd c(3);
        // double constant = refresh_rate * a;
        c << pow(time_period, 2) / 2, pow(time_period, 3) / 6, 5 * pow(time_period, 4) / 24;
        return x.transpose() * c + s_dot_dot * time_period + s_dot;
    }

    inline double accelerationAvg(VectorXd &x, double s_dot_dot){
        double sum = 0;
        for(short a = 1; a <= num_points; a++){
            VectorXd c(3);
            double constant = refresh_rate * a;
            c << constant, pow(constant, 2) / 2, pow(constant, 3) / 6;
            sum +=  x.transpose() * c + s_dot_dot;
        }
        return (sum / double(num_points));
    }

    inline double accelerationF(VectorXd &x, double constant, double s_dot_dot){
        VectorXd c(3);
        c << constant, pow(constant, 2) / 2, pow(constant, 3) / 6;
        return x.transpose() * c + s_dot_dot;
    }

    inline double jerkF(double average_acceleration){
        /*
        VectorXd c(3);
        c << 1 , constant, pow(constant, 2) / 2;
        return x.transpose() * c;
        */
        return (average_acceleration / refresh_rate);
    }

    inline double squareJerk(VectorXd &x, double constant){
        VectorXd c(3);
        c << 1 , constant, pow(constant, 2) / 2;
        return pow(x.transpose() * c, 2);
    }

    // cost functions
    inline double costV(short lane, double time, double diff, VectorXd &calculated){
        cout << "behavior_planner::costS_Vel\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kv = " << k_v << ", diff^2 = " << pow(diff, 2)
             << " ]" << endl;
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_v * pow(diff, 2);
    }

    inline double costS(short lane, double time, double diff, VectorXd &calculated){
        cout << "behavior_planner::costS\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", ks = " << k_s << ", diff^2 = " << pow(diff, 2)
             << " ]" << endl;
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_s * pow(diff, 2) + k_s_bias;
    }

    inline double costD(short lane, double time, double diff, VectorXd &calculated){
        cout << "\tbehavior_planner::costD\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2)
             << " ]" << endl;
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_d * pow(diff, 2);
    }

    // behavior based on velocity
    void calculateHighwayS(short lane, double &cost, VectorXd &s);

    void calculateHighwayD(short lane, double &score_d, VectorXd &d);

    inline void sharedCalc(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, VectorXd &c){
        // cout << "sharedCalc: [ time = " << time << ", x = " << x << ", x_dot = " << x_dot << ", x_dot_dot = " << x_dot_dot << ", xf = " << xf << ", xf_dot = " << xf_dot << ", xf_dot_dot = " << xf_dot_dot << endl;
        // cout << "\nA^-1:\n" << Ai << endl;

        MatrixXd A = MatrixXd::Zero(3, 3);
        VectorXd B = VectorXd::Zero(3);

        A << pow(time, 3), pow(time, 4), pow(time, 5),
                3 * pow(time, 2), 4 *pow(time, 3), 5 * pow(time, 4),
                6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

        B << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
                (xf_dot - (x_dot + x_dot_dot * time)),
                (xf_dot_dot - x_dot_dot);

        // cout << "B = " << B.transpose() << endl;

        c = A.inverse() * B;
        // cout << c.transpose() << endl;
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
