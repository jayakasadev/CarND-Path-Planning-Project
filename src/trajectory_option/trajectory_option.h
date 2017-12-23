//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_OPTION_H
#define PATH_PLANNING_TRAJECTORY_OPTION_H

#include <limits>
#include "../Eigen-3.3/Eigen/Dense"

class trajectory_option{
private:
    bool first;
public:
    Eigen::VectorXd * vector;
    double score;
    double time;
    // double xf;
    // double xf_dot;
    // double xf_dot_dot;

    trajectory_option(){
        first = true;
    }

    ~trajectory_option(){}

    void reset(double x, double x_dot, double x_dot_dot){
        // std::cout << "trajectory_option::reset";
        if(!first){
            delete vector;
        }
        first = false;
        score = 100000.0d;
        time = 0.0d;
        // xf = 0;
        // xf_dot = 0;
        // xf_dot_dot = 0;

        vector = new Eigen::VectorXd(6);
        (*vector)[0] = x;
        (*vector)[1] = x_dot;
        (*vector)[2] = x_dot_dot;
        // std::cout << (*vector).transpose()  << std::endl;
    }

    void update(Eigen::VectorXd &xf, double time, double score){
        // std::cout << "trajectory_option::update\tscore:" << score << "\ttime:" << time << "\t";
        if(!first) {
            (*vector)[3] = xf[0];
            (*vector)[4] = xf[1];
            (*vector)[5] = xf[2];

            this->time = time;
            this->score = score;
            // std::cout << (*vector).transpose();
        }
        // std::cout << std::endl;
    }

    void print(){
        std::cout << "vector: "  << (*vector).transpose() << "\ttime: " << time << "\tscore: " << score << std::endl;
    }
};

#endif //PATH_PLANNING_TRAJECTORY_OPTION_H
