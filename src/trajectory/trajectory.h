#ifndef PATH_PLANNING_TRAEJCTORY_H
#define PATH_PLANNING_TRAEJCTORY_H

#include <iostream>
#include "../enums/trajectory_type.h"
#include "../Eigen-3.3/Eigen/Dense"
#include "../constants/road_constants.h"

class trajectory{
private:
    trajectory_type type;
    Eigen::VectorXd *vector;
    short count;
    double time;
    double score;
    bool invalid;
    double prev_point;

    inline bool isInvalid(){
        return this->invalid;
    }

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
    trajectory(trajectory_type type, double time){
        // std::cout << "trajectory constructor\ttime: " << time << std::endl;
        this->type = type;
        this->vector = new Eigen::VectorXd(6);
        this->count = 0;
        this->invalid = false;
        this->time = time;
    }

    ~trajectory(){
        // std::cout << "trajectory destructor\ttime: " << time << std::endl;
        delete vector;
    }

    /**
     * Method to calculate the next n points needed.
     *
     * @param diff
     * @return vector of next n points
     */
    std::vector<double> getPoints(short n);

    inline Eigen::VectorXd getVector(){
        return *vector;
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

    inline void setInvalid(){
        this->invalid = true;
    }

    inline void reset(){
        this->count = 0;
        this->invalid = false;
    }

    void print();
};

#endif //PATH_PLANNING_TRAEJCTORY_H
