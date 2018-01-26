#ifndef PATH_PLANNING_TRAEJCTORY_H
#define PATH_PLANNING_TRAEJCTORY_H

#include <iostream>
#include <memory>
#include "../enums/trajectory_type.h"
#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/simulator_constants.h"
#include "../utilities/unique_ptr_helper.h"

class trajectory{
private:
    trajectory_type type;
    Eigen::VectorXd vector;
    short count;
    double time;
    double score;
    double prev_point;

    /**
     * Method to check if this trajectory has points available
     *
     * @param diff
     * @return true if there are enough points, else false
     */
    inline bool gotPoints(short diff){
        return (time / refresh_rate) > (count + diff);
    }

public:
    trajectory(){
        // std::cout << "trajectory constructor\ttime: " << time << std::endl;
        this->vector = Eigen::VectorXd::Zero(6);
        this->count = 0;
        this->score = 0;
    }

    trajectory(const trajectory &trajectory){
        // std::cout << "trajectory copy constructor\ttime: " << time << std::endl;
        this->vector = trajectory.vector;
        this->count = trajectory.count;
        this->type = trajectory.type;
        this->time = trajectory.time;
        this->score = trajectory.score;
        this->prev_point = trajectory.prev_point;
    }

    ~trajectory(){
        std::cout << "trajectory destructor\ttime: " << time << std::endl;
    }

    inline void setType(trajectory_type type){
        this->type = type;
    }

    inline void setTime(double time){
        this->time = time;
    }

    /**
     * Method to calculate the next n points needed.
     *
     * @param diff
     * @return vector of next n points
     */
    std::vector<double> getPoints(short n);

    inline Eigen::VectorXd getVector(){
        return vector;
    }

    inline trajectory_type getType(){
        return this->type;
    }

    inline double getScore(){
        return this->score;
    }

    inline void addScore(double subscore){
        this->score += subscore;
    }

    inline bool isValid(){
        return abs(vector[1]) < max_velocity_mps && abs(vector[2]) < max_acceleration && abs(vector[3]) < max_jerk;
    }

    inline void reset(){
        this->count = 0;
        this->score = 0;
    }

    /**
     * Method to print out option information
     *
     * @param os
     * @param obj
     * @return ostream to print object
     */
    friend std::ostream& operator <<(std::ostream& os, trajectory& obj){
        os << "trajectory: " << obj.vector.transpose() << "\nvalidity: " << obj.isValid() << "\nscore: "
           << obj.score << "\ntime: " << obj.time << "\tcount: " << obj.count;
        return os;
    }
};

#endif //PATH_PLANNING_TRAEJCTORY_H
