//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"

void behavior_planner::calculateS(short lane, double &score_s, VectorXd &s){
    double sf = 0, sf_dot = 0, sf_dot_dot = 0;
    getSfVals(sf, sf_dot, lane);

    for(double time = 1.0; time <= 5.0; time+=refresh_rate) {
        // cout << "time: " << time;
        // cout << "CALCULATE S" << endl;// generate matrices
        VectorXd c_s = VectorXd::Zero(3);
        sharedCalc(time, car->getS(), car->getVelocityS(), car->getAccelerationS(), sf, sf_dot, sf_dot_dot, c_s);
        // cout << c_s.transpose() << endl;
        // cout << "\tS -> jerk: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << endl;

        // calculate cost
        // cout << "A ";
        if (c_s[0] < max_jerk && c_s[0] > -max_jerk) {
            // cout << "B ";
            if(sf_dot > car->getVelocityS() && c_s[0] < 0){
                break; // skip and jerk values that have me moving in the wrong direction
            }
            if(sf_dot < car->getVelocityS() && c_s[0] > 0){
                break; // skip and jerk values that have me moving in the wrong direction
            }
            if(sf_dot == car->getVelocityS() && c_s[0] != 0){
                break; // skip and jerk values that have me moving in the wrong direction
            }

            float cost;
            if (values->getBehavior(lane) == KEEP_VELOCITY) {
                // cout << "C ";
                cost = costS(lane, time, sf, c_s);
            } else {
                // cout << "D ";
                cost = costS(lane, time, sf_dot, c_s);
            }
            // cout << "E ";
            if (cost < score_s) {
                // cout << "F " << endl;
                score_s = cost;
                // cout << "cost: " << cost << endl;
                VectorXd s_temp = VectorXd::Zero(6);
                s_temp << car->getS(), car->getVelocityS(), car->getAccelerationS(), c_s[0], c_s[1], c_s[2];
                s = s_temp;
                // cout << "S: " << s.transpose() << endl;
            }
            // cout << "G ";
        }else {
            if((abs(c_s[0]) - max_jerk) > 5){
                time += refresh_rate; // speed up the calculations
            }
        }
        // cout << endl;
    }
}

void behavior_planner::calculateD(short lane, double &score_d, VectorXd &d){
    double df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;

    for(double time = 1.0; time <= 5.0; time+=refresh_rate){
        // cout << "time: " << time;

        VectorXd c_d = VectorXd::Zero(3);
        // cout << "CALCULATE D" << endl;
        sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot, c_d);
        // cout << c_d.transpose() << endl;

        // cout << "\tD -> jerk: " << c_d[0] << " snap: " << c_d[1] << " crackle: " << c_d[2] << endl;

        double diff = df - car->getD();
        // calculate cost
        if(c_d[0] < max_jerk && c_d[0] > -max_jerk){
            if(car->getLane() < lane && c_d[0] < 0){ // turn right
                break; // skip and jerk values that have me moving in the wrong direction
            }
            if(car->getLane() > lane && c_d[0] > 0){ // turn left
                break; // skip and jerk values that have me moving in the wrong direction
            }
            if(car->getLane() == lane && c_d[0] != 0){ // stay in lane
                break; // skip and jerk values that have me moving in the wrong direction
            }

            float cost = costD(lane, time, diff, c_d);
            if(cost < score_d){
                score_d = cost;
                // cout << "cost: " << cost << endl;
                VectorXd d_temp = VectorXd::Zero(6);
                d_temp  << car->getD(), car->getVelocityD(), car->getAccelerationD(), c_d[0], c_d[1], c_d[2];
                d = d_temp;
                // cout << "D: " << d.transpose() << endl;
            }
        }
        else {
            if((abs(c_d[0]) - max_jerk) > 5){
                time += refresh_rate; // speed up the calculations
            }
        }
    }
}

trajectory_option* behavior_planner::highwayPlanning(short lane){
    trajectory_option* option = new trajectory_option();
    // calculate sf values
    // cout << "s: " << car->getS() << " s_dot: " << car->getVelocityS() << " s_dot_dot: " << car->getAccelerationS() << endl;
    // cout << "sf: " << sf << " sf_dot: " << sf_dot << " sf_dot_dot: " << sf_dot_dot << endl;

    future<void> calc_s = async(launch::deferred, [this, lane, option]{ calculateS(lane, option->score_s, option->s);});
    future<void> calc_d = async(launch::deferred, [this, lane, option]{ calculateD(lane, option->score_d, option->d);});

    calc_s.get();
    calc_d.get();
    // cout << "highwayPlanning: lowest s score so far: " << option->score_s << "\t\tlowest d score so far: " << option->score_d << endl;
    return option;
}

trajectory_option behavior_planner::cityPlanning(short lane){
    cout << "behavior_planner::cityPlanning" << endl;
    trajectory_option option;
    // calculate for s and d together by calculating the d first and using the selected time t to calculate s as well after

    return option;
}

vector<VectorXd> behavior_planner::bestOption(){
    cout << "behavior_planner::bestOption" << endl;
    vector<future<trajectory_option>> options;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        // TODO setup city planning code
        if(0 <= values->getVelocity(a)){
            options.push_back(async(launch::deferred, [this, a]{return *this->highwayPlanning(a);}));
        } else {
            // city planning
            // do nothing for now
            // options.push_back(async(launch::async, [this, &a]{return this->cityPlanning(a);}));
        }
    }

    // cout << "finished calculations: " << options.size() << endl;

    // this part is synchronous
    // cannot compare without the completed computations
    // this part is only as slow as the slowest calculation

    trajectory_option lowest = options[0].get();
    VectorXd s  = lowest.s;
    VectorXd d  = lowest.d;
    double score_s = lowest.score_s;
    double score_d = lowest.score_d;
    cout << "lowest s score so far: " << score_s << "\t\tlowest d score so far: " << score_d << endl;
    for(short a = 1; a < options.size(); a++){
        cout << "a: " << a << endl;
        try{
            lowest = options[a].get();
            if(score_s > lowest.score_s && score_d > lowest.score_d){
                score_s = lowest.score_s;
                s = lowest.s;

                score_d = lowest.score_d;
                d = lowest.d;
            }
            cout << "lowest s score so far: " << score_s << "\t\tlowest d score so far: " << score_d << endl;
            // cout << "comparing it to this score: " << compare.score << endl;
        } catch (future_error &e){
            cout << e.what() << endl;
        }
    }

    // cout << "picked the best option" << endl;
    cout << "S: " << s.transpose() << endl;
    cout << "D: " << d.transpose() << endl;

    return {s, d};
}