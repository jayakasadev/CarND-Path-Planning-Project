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

    // weights for the cost function
    // max value is 7500 for a bad score
    // best score is 0

    const float k_j = 1;

    const float k_d = 39.0625;

    const float k_t = 156.25;

    const float k_s = -4;

    const float k_s_bias = 2500;

    const float k_v = 25;

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
            sf = car->getS() + values->getDistanceFront(lane);
        } else if(behavior == vehicle_behavior::FOLLOW) {
            // std::cout << "FOLLOW " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() + values->getDistanceFront(lane) + values->getVelocity(lane) * time_period;
            sf_dot = values->getVelocity(lane);
        } else if(behavior == vehicle_behavior::MERGE){
            // std::cout << "MERGE " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() + values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        } else{
            // std::cout << "KEEP VELOCITY " << " s: " << car->getS() << " d: " << values->getDistanceFront(lane);
            sf = car->getS() +  + values->getDistanceFront(lane);
            sf_dot = values->getVelocity(lane);
        }
        // std::cout << " sf: " << sf << " sf_dot: " << sf_dot << std::endl;
    }

    inline double positionF(Eigen::VectorXd &x, double constant, double s, double s_dot, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        c << pow(constant, 3) / 6, pow(constant, 4) / 24, pow(constant, 5) / 120;
        return x.transpose() * c + .5 * s_dot_dot * pow(constant, 2) + s_dot * constant + s;
    }

    inline double velocityF(Eigen::VectorXd &x, double constant, double s_dot, double s_dot_dot){
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
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        // double constant = refresh_rate * a;
        c << pow(constant, 2) / 2, pow(constant, 3) / 6, 5 * pow(constant, 4) / 24;
        return x.transpose() * c + s_dot_dot * constant + s_dot;
    }

    inline double accelerationAvg(Eigen::VectorXd &x, double s_dot_dot){
        double sum = 0;
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        for(short a = 1; a <= num_points; a++){
            double constant = refresh_rate * a;
            c << constant, pow(constant, 2) / 2, pow(constant, 3) / 6;
            sum +=  x.transpose() * c + s_dot_dot;
        }
        return (sum / double(num_points));
    }

    inline double accelerationF(Eigen::VectorXd &x, double constant, double s_dot_dot){
        Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
        c << constant, pow(constant, 2) / 2, pow(constant, 3) / 6;
        return x.transpose() * c + s_dot_dot;
    }

    inline double jerkAvg(double average_acceleration){
        /*
        VectorXd c(3);
        c << 1 , constant, pow(constant, 2) / 2;
        return x.transpose() * c;
        */
        return (average_acceleration / refresh_rate);
    }

    inline double squareJerk(Eigen::VectorXd &x, double constant){
        Eigen::VectorXd c(3);
        c << 1 , constant, pow(constant, 2) / 2;
        return pow(x.transpose() * c, 2);
    }

    // cost functions
    inline double costV(double time, double diff, Eigen::VectorXd &calculated){
        /*
        std::cout << "planner::costS_Vel\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kv = " << k_v << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_v * pow(diff, 2);
    }

    inline double costS(double time, double diff, Eigen::VectorXd &calculated){
        /*
        std::cout << "planner::costS\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", ks = " << k_s << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_s * pow(diff, 2) + k_s_bias;
    }

    inline double costD(double time, double diff, Eigen::VectorXd &calculated){
        /*
        std::cout << "\tplanner::costD\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (time - time_period) + k_d * pow(diff, 2);
    }

    inline double costSD(double time, double diff_s, double diff_d, Eigen::VectorXd &calculated){
        /*
        std::cout << "\tplanner::costD\t[ k_j = " << k_j << ", jerk^2 = " << squareJerk(calculated, time_period)
             << ", kt = " << k_t << ", time = " << (time - time_period) << ", kd = " << k_d << ", diff^2 = " << pow(diff, 2)
             << " ]" << std::endl;
        */
        // cout << calculated.transpose() << endl;
        return k_j * squareJerk(calculated, time_period) + k_t * (diff_s) + k_d * pow(diff_d, 2);
    }

    inline Eigen::VectorXd sharedCalc(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot){
        /*
        std::cout << "sharedCalc: [ time = " << time << ", x = " << x << ", x_dot = " << x_dot << ", x_dot_dot = "
                  << x_dot_dot << ", xf = " << xf << ", xf_dot = " << xf_dot << ", xf_dot_dot = " << xf_dot_dot
                  << std::endl;
        */
        // std::cout << "\nA^-1:\n" << Ai << std::endl;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
        Eigen::VectorXd B = Eigen::VectorXd::Zero(3);

        A << pow(time, 3), pow(time, 4), pow(time, 5),
                3 * pow(time, 2), 4 *pow(time, 3), 5 * pow(time, 4),
                6 * time, 12 * pow(time, 2), 20 * pow(time, 3);

        B << (xf - (x + x_dot * time + 0.5 * x_dot_dot * pow(time, 2))),
                (xf_dot - (x_dot + x_dot_dot * time)),
                (xf_dot_dot - x_dot_dot);

        // std::cout << "B = " << B.transpose() << std::endl;

        return A.inverse() * B;
        // std::cout << c.transpose() << std::endl;
    }
};

#endif //PATH_PLANNING_PLANNER_H
