//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"

trajectory_option behavior_planner::highwayPlanning(short lane){
    // println("behavior_planner::highwayPlanning ");
    // cout << lane << endl;
    // values->printScores();
    // car->print();
    trajectory_option option;
    // calculate for s and d separately for each time t --> more costly
    double sf = 0, sf_dot = 0, sf_dot_dot = 0, df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;
    for(short a = 0; a <= num_points; a++){
        double time = time_period + refresh_rate * a;
        // cout << "time: " << time << endl;
        // calculate sf values
        getSfVals(sf, sf_dot, lane);
        // cout << "s: " << car->getS() << " s_dot: " << car->getVelocityS() << " s_dot_dot: " << car->getAccelerationS() << endl;
        // cout << "sf: " << sf << " sf_dot: " << sf_dot << " sf_dot_dot: " << sf_dot_dot << endl;
        // generate matrices
        VectorXd c_s = VectorXd::Zero(3);
        // cout << "CALCULATE S" << endl;
        sharedCalc(time,car->getS(), car->getVelocityS(), car->getAccelerationS(), sf, sf_dot, sf_dot_dot, c_s);
        // cout << c_s.transpose() << endl;
        if(c_s[0] >= max_jerk){
            continue;
        }

        // calculate cost
        float cost;
        if(values->getBehavior(lane) == KEEP_VELOCITY){
            cost = costS(lane, time, sf, c_s);
        } else {
            cost = costS(lane, time, sf_dot, c_s);
        }
        if(cost < option.score_s){
            option.score_s = cost;

            VectorXd s = VectorXd::Zero(6);
            s << car->getS(), car->getVelocityS(), car->getAccelerationS(), c_s[0], c_s[1], c_s[2];
            option.s = s;
            // cout << "S: " << s.transpose() << endl;
        }
        cout << "jerk for S: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << " cost: " << cost << endl;

        VectorXd c_d = VectorXd::Zero(3);
        // cout << "CALCULATE D" << endl;
        sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot, c_d);
        // cout << c_d.transpose() << endl;

        if(c_d[0] >= max_jerk){
            continue;
        }

        double diff = df - car->getD();
        // calculate cost
        cost = costD(lane, time, diff, c_d);
        if(cost < option.score_d){
            option.score_d = cost;

            VectorXd d = VectorXd::Zero(6);
            d << car->getD(), car->getVelocityD(), car->getAccelerationD(), c_d[0], c_d[1], c_d[2];
            option.d = d;
            // cout << "D: " << d.transpose() << endl;
        }

        cout << "jerk for D: " << c_d[0] << " snap: " << c_d[1] << " crackle: " << c_d[2] << " cost: " << cost << endl;
    }

    // cout << "done" << endl;
    return option;
}

trajectory_option behavior_planner::cityPlanning(short lane){
    cout << "behavior_planner::cityPlanning" << endl;
    trajectory_option option;
    // calculate for s and d together by calculating the d first and using the selected time t to calculate s as well after

    return option;
}

vector<VectorXd> behavior_planner::bestOption(){
    // println("behavior_planner::bestOption");
    vector<future<trajectory_option>> options;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        if(0 <= values->getVelocity(a)){ // TODO setup city planning code
            options.push_back(async(launch::deferred, [this, a]{return this->highwayPlanning(a);}));
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

    for(short a = 1; a < options.size(); a++){
        // cout << "index: " << index << " a: " << a << endl;
        try{
            trajectory_option compare = options[a].get();
            if(lowest.score_s > compare.score_s){
                lowest.score_s = compare.score_s;
                lowest.s = compare.s;
            }

            if(lowest.score_d > compare.score_d){
                lowest.score_d = compare.score_d;
                lowest.d = compare.d;
            }
            cout << "lowest s score so far: " << lowest.score_s << "\t\tlowest d score so far: " << lowest.score_d << endl;
            // cout << "comparing it to this score: " << compare.score << endl;
        } catch (future_error &e){
            cout << e.what() << endl;
        }
    }

    // cout << "picked the best option" << endl;

    return {lowest.s, lowest.d};
}