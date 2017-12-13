//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"

double behavior_planner::cost(short lane, VectorXd &calculated){
    // println("behavior_planner::cost");
    return  0;
}

double behavior_planner::costSport(short lane, VectorXd &calculated){
    // println("behavior_planner::costSport");
    return  0;
}

double behavior_planner::costEconomy(short lane, VectorXd &calculated){
    // println("behavior_planner::costEconomy");
    return  0;
}


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
        VectorXd c_s(3);
        sharedCalc(time,car->getS(), car->getVelocityS(), car->getAccelerationS(), sf, sf_dot, sf_dot_dot, c_s);

        VectorXd c_d(3);
        sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot, c_d);

        // calculate cost
        // cost(lane, c);
        cout << "jerk for S: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << endl;
        cout << "jerk for D: " << c_d[0] << " snap: " << c_d[1] << " crackle: " << c_d[2] << endl;
    }

    cout << "done" << endl;
    return option;
}

trajectory_option behavior_planner::cityPlanning(short lane){
    cout << "behavior_planner::cityPlanning" << endl;
    trajectory_option option;
    // calculate for s and d together
    return option;
}

vector<VectorXd> behavior_planner::bestOption(){
    // println("behavior_planner::bestOption");
    vector<future<trajectory_option>> options;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        short lane = a;
        // highway planning
        if(velocity_barrier <= values->getVelocity(a)){
            options.push_back(async(launch::async, [this, &lane]{return this->highwayPlanning(lane);}));
        } else {
            // city planning
            // do nothing for now
            // options.push_back(async(launch::async, [this, &a]{return this->cityPlanning(a);}));
        }
    }

    cout << "finished calculations: " << options.size() << endl;

    // this part is synchronous
    // cannot compare without the completed computations
    // this part is only as slow as the slowest calculation

    trajectory_option lowest = options[0].get();

    for(short a = 1; a < options.size(); a++){
        // cout << "index: " << index << " a: " << a << endl;
        try{
            trajectory_option compare = options[a].get();
            if(lowest.score > compare.score){
                lowest = compare;
            }
            // cout << "lowest score so far: " << lowest.score << endl;
            // cout << "comparing it to this score: " << compare.score << endl;
        } catch (future_error &e){
            println(e.what());
        }
    }

    cout << "picked the best option" << endl;

    return {lowest.s, lowest.d};
}