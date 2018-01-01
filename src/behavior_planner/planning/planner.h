#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <functional>
#include <iostream>

#include "../../constants/constants.h"
#include "../../Eigen-3.3/Eigen/Dense"
#include "../../scores/scores.h"
#include "../../vehicle/driver.h"
#include "../../trajectory_option/trajectory_option.h"

using namespace Eigen;
using namespace std;

class planner{

public:
    trajectory_option option_s;
    trajectory_option option_d;

    // double s_v;
    // double s_a;
    // double d_v;
    // double d_a;

    planner(){
        // setting the score function based on drive mode
        if(driveMode == REGULAR){
            scoreFunction = [](double score){ return scalar * pow(score - 3750, 2);}; // favor median scores
        } else if(driveMode == SPORT){
            scoreFunction = [](double score){ return -score + 7500;}; // favor larger legal scores
        } else {
            // score function does nothing special
            scoreFunction = [](double score){ return score;};
        }
    }

    ~planner(){}

protected:
    std::function<double(double)> scoreFunction;

    driver *car;
    scores *values;
    short lane;

    inline void getSfVals(double &sf, double &sf_dot, short lane){
        // values->printScores();
        // car->print();
        // std::cout << "behavior: " << values->getBehavior(lane);
        vehicle_behavior behavior = values->getBehavior(lane);
        if(behavior == vehicle_behavior::STOP) {
            // std::cout << "STOP " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = values->getDistanceFront(lane);
            sf_dot = 0;
        } else if(behavior == vehicle_behavior::FOLLOW) {
            // std::cout << "FOLLOW " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = values->getDistanceFront(lane) + values->getVelocity(lane) * time_period - follow_buffer;
            sf_dot = values->getVelocity(lane);
        } else if(behavior == vehicle_behavior::MERGE){
            // std::cout << "MERGE " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        } else{
            // std::cout << "KEEP VELOCITY " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        }
        // std::cout << " sf: " << sf << " sf_dot: " << sf_dot << std::endl;
    }

    inline double positionF(Eigen::VectorXd &x, double constant, double s, double s_dot, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        c << pow(constant, 3.0d) / 6.0d, pow(constant, 4.0d) / 24.0d, pow(constant, 5.0d) / 120.0d;
        return x.transpose() * c + s_dot_dot * pow(constant, 2.0d) / 2.0d + s_dot * constant + s;
    }

    inline bool moveForwardOnly(Eigen::VectorXd &x, double s, double s_dot, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        for(short a = 1; a <= num_points; a++){
            double constant = refresh_rate * double(a);
            c << pow(constant, 3.0d) / 6.0d, pow(constant, 4.0d) / 24.0d, pow(constant, 5.0d) / 120.0d;
            double acc = x.transpose() * c + s_dot_dot * pow(constant, 2.0d) / 2.0d + s_dot * constant + s;
            if((acc - s) <= 0) return false; // do not want any weird skips in my path for any reason
        }
        return true;
    }

    inline double velocityAvg(Eigen::VectorXd &x, double s_dot, double s_dot_dot){
        double sum = 0;
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        for(short a = 1; a <= num_points; a++){
            double constant = refresh_rate * double(a);
            c << pow(constant, 2.0d) / 2.0d, pow(constant, 3.0d) / 6.0d, pow(constant, 4.0d) / 24.0d;
            double acc = x.transpose() * c + s_dot_dot * constant + s_dot;
            if(abs(acc) >= max_velocity) return acc; // do not want any weird skips in my path for any reason
            sum += abs(acc);
        }
        return (sum / double(num_points));
    }

    inline double velocityF(Eigen::VectorXd &x, double constant, double s_dot, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        // double constant = refresh_rate * a;
        c << pow(constant, 2.0d) / 2.0d, pow(constant, 3.0d) / 6.0d, pow(constant, 4.0d) / 24.0d;
        return x.transpose() * c + s_dot_dot * constant + s_dot;
    }

    inline double accelerationAvg(Eigen::VectorXd &x, double s_dot_dot){
        double sum = 0;
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        for(short a = 1; a <= num_points; a++){
            double constant = refresh_rate * double(a);
            c << constant, pow(constant, 2.0d) / 2.0d, pow(constant, 3.0d) / 6.0d;
            double acc = x.transpose() * c + s_dot_dot;
            if(abs(acc) >= max_acceleration) return acc; // do not want any weird skips in my path for any reason
            sum += abs(acc);
        }
        return (sum / double(num_points));
    }

    inline double accelerationF(Eigen::VectorXd &x, double constant, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        c << constant, pow(constant, 2.0d) / 2.0d, pow(constant, 3.0d) / 6.0d;
        return x.transpose() * c + s_dot_dot;
    }

    inline double jerkAvg(double average_acceleration){
        return (average_acceleration / refresh_rate);
    }

    inline double squareJerk(Eigen::VectorXd &x, double constant){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        c << 1.0d , constant, pow(constant, 2.0d) / 2.0d;
        return pow(x.transpose() * c, 2.0d);
    }

    inline VectorXd sharedCalc(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot){
        /*
        std::cout << "sharedCalc: [ time = " << time << ", x = " << x << ", x_dot = " << x_dot << ", x_dot_dot = "
                  << x_dot_dot << ", xf = " << xf << ", xf_dot = " << xf_dot << ", xf_dot_dot = " << xf_dot_dot
                  << std::endl;
        */
        // std::cout << "\nA^-1:\n" << Ai << std::endl;

        MatrixXd A = MatrixXd::Zero(3, 3);
        VectorXd B = VectorXd::Zero(3);

        A << pow(time, 3), pow(time, 4), pow(time, 5),
                3.0d * pow(time, 2), 4.0d *pow(time, 3), 5.0d * pow(time, 4),
                6.0d * time, 12.0d * pow(time, 2), 20.0d * pow(time, 3);

        B << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
                (xf_dot - (x_dot + x_dot_dot * time)),
                (xf_dot_dot - x_dot_dot);

        // std::cout << "B = " << B.transpose() << std::endl;

        return A.inverse() * B;
        // std::cout << c.transpose() << std::endl;
    }
};

#endif //PATH_PLANNING_PLANNER_H
