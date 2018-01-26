#ifndef PATH_PLANNING_TRAJECTORY_OPTION_H
#define PATH_PLANNING_TRAJECTORY_OPTION_H

#include <iostream>
#include "../../Eigen-3.3/Eigen/Dense"

class trajectory_calculator{
private:
    Eigen::MatrixXd t; // matrix holds time values
    double time;
public:

    trajectory_calculator(){
        // std::cout << "trajectory_calculator default constructor" << std::endl;
        // generate the time matrix
        t = Eigen::MatrixXd::Zero(3,3);
        time = -1;
    }

    // the following lets me make use of these objects within vectors

    // copy constructor
    trajectory_calculator(const trajectory_calculator &calculator){
        // std::cout << "trajectory_calculator copy constructor" << std::endl;
        this->t = calculator.t;
        this->time = time;
    }

    // assignment operator override
    trajectory_calculator& operator=(const trajectory_calculator &calculator){
        // std::cout << "trajectory_calculator assignment" << std::endl;
        if(this != &calculator){
            this->t = calculator.t;
        }
        return *this;
    }

    ~trajectory_calculator(){
        std::cout << "trajectory_calculator destructor" << std::endl;
    }

    void initialize(double time);

    /**
     * Method to perform calculations for trajectory option in a lazy fashion. Useful in a memopry restricted environment
     *
     * @param time
     * @param x
     * @param x_dot
     * @param x_dot_dot
     * @param xf
     * @param xf_dot
     * @param xf_dot_dot
     * @param vector
     */
    void calculate(double time, double x, double x_dot, double x_dot_dot, double xf, double xf_dot, double xf_dot_dot, Eigen::VectorXd &vector);

    /**
     * Method to print out option information
     *
     * @param os
     * @param obj
     * @return ostream to print object
     */
    friend std::ostream& operator <<(std::ostream& os, const trajectory_calculator& obj){
        os << "trajectory_calculator: time: " << obj.time << "\n" << obj.t;
        return os;
    }
};

#endif //PATH_PLANNING_TRAJECTORY_OPTION_H
