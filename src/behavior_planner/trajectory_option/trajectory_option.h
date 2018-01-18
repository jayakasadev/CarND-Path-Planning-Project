#ifndef PATH_PLANNING_TRAJECTORY_OPTION_H
#define PATH_PLANNING_TRAJECTORY_OPTION_H

#include <iostream>
#include "../../Eigen-3.3/Eigen/Dense"

class trajectory_option{
private:
    double time;
    Eigen::MatrixXd * t; // matrix holds time values

public:

    trajectory_option(double time){
        std::cout << "trajectory_option constructor\ttime: " << time << std::endl;
        this->time = time;
        // generate the time matrix
        t = new Eigen::MatrixXd(3,3);
        *t << pow(time, 3), pow(time, 4), pow(time, 5),
                3.0d * pow(time, 2), 4.0d *pow(time, 3), 5.0d * pow(time, 4),
                6.0d * time, 12.0d * pow(time, 2), 20.0d * pow(time, 3);
        // std::cout << *t << std::endl;
        *t = t->inverse();
        // std::cout << "inverted: " << std::endl;
        // std::cout << *t << std::endl;
    }

    ~trajectory_option(){
        std::cout << "trajectory_option destructor\ttime: " << time << std::endl;
        delete t;
    }

    /**
     *
     * Method to perform calculations for trajectory option
     *
     * @param x
     * @param x_dot
     * @param x_dot_dot
     * @param xf
     * @param xf_dot
     * @param xf_dot_dot
     * @param vector
     */
    void calculate(double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, Eigen::VectorXd &vector);

    /**
     * Method to print out option information
     */
    void print();
};

#endif //PATH_PLANNING_TRAJECTORY_OPTION_H
