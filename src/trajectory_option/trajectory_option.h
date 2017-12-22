//
// Created by jay on 12/21/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_OPTION_H
#define PATH_PLANNING_TRAJECTORY_OPTION_H

#include <limits>
#include "../Eigen-3.3/Eigen/Dense"

struct trajectory_option{
public:
    Eigen::VectorXd * s;
    Eigen::VectorXd * d;
    double scoreS;
    double scoreD;
    double timeS;
    double timeD;

    trajectory_option(){

        scoreS = 100000.0d;
        scoreD = 100000.0d;

        timeS = 0;
        timeD = 0;
    }

    ~trajectory_option(){}
};

#endif //PATH_PLANNING_TRAJECTORY_OPTION_H
