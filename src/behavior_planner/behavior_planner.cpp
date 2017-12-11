//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"

trajectory_option behavior_planner::calculate(short lane){
    cout << "behavior_planner::calculate" << endl;
    trajectory_option opt;
    return opt;
}

trajectory_option behavior_planner::calculateSport(short lane){
    cout << "behavior_planner::calculateSport" << endl;
    trajectory_option opt;
    return opt;
}

trajectory_option behavior_planner::calculateEconomy(short lane){
    cout << "behavior_planner::calculateEconomy" << endl;
    trajectory_option opt;
    return opt;
}

vector<VectorXd> behavior_planner::bestOption(){
    vector<future<trajectory_option>> options;
    for(short a = 0; a < num_lanes; a++){
        // future<trajectory_option> future_option;
        if(driveMode == SPORT){
            cout << "SPORT" << endl;
            options.push_back(async([this, a]{ return this->calculateSport(a); }));
        } else if(driveMode == ECONOMY){
            cout << "ECONOMY" << endl;
            options.push_back(async([this, a]{ return this->calculateEconomy(a); }));
        } else {
            // cout << "REGULAR" << endl;
            options.push_back(async([this, a]{ return this->calculate(a); }));
        }
    }

    // cout << "finished calculations: " << options.size() << endl;

    short index = 0;
    trajectory_option lowest = options[index].get();

    for(short a = 1; a < num_lanes; a++){
        // cout << "index: " << index << " a: " << a << endl;
        try{
            trajectory_option compare = options[a].get();
            if(lowest.score > compare.score){
                index = a;
                lowest = compare;
            }
            // cout << "lowest score so far: " << lowest.score << endl;
            // cout << "comparing it to this score: " << compare.score << endl;
        } catch (future_error &e){
            cout << e.what() << endl;
        }
    }

    // cout << "picked the best option" << endl;

    return {lowest.s, lowest.d};
}