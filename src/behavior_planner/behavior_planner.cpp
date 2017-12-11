//
// Created by jay on 12/5/17.
//

#include "behavior_planner.h"

trajectory_option behavior_planner::calculate(short lane){
    trajectory_option opt;
    return opt;
}

trajectory_option behavior_planner::calculateSport(short lane){
    trajectory_option opt;
    return opt;
}

trajectory_option behavior_planner::calculateEconomy(short lane){
    trajectory_option opt;
    return opt;
}

vector<VectorXd> behavior_planner::bestOption(){
    vector<future<trajectory_option>> options;
    for(short a = 0; a < num_lanes; a++){
        // future<trajectory_option> future_option;
        if(driveMode == SPORT){
            options.push_back(async([this, a]{ return this->calculateSport(a); }));
        } else if(driveMode == ECONOMY){
            options.push_back(async([this, a]{ return this->calculateEconomy(a); }));
        } else {
            options.push_back(async([this, a]{ return this->calculate(a); }));
        }
    }

    cout << "finished calculations" << endl;

    short index = 0;

    for(short a = 1; a < num_lanes; a++){
        cout << "index: " << index << " a: " << a << endl;
        float lowest = 0, score = 0;
        try{
            lowest = options[index].get().score;
            score = options[a].get().score;
            if(lowest > score){
                index = a;
            }
            cout << "lowest score so far: " << lowest << endl;
            cout << "comparing it to this score: " << score << endl;
        } catch (future_error &e){
            cout << e.what() << endl;
        }
    }

    cout << "picked the best option" << endl;

    return {options[index].get().s, options[index].get().d};
}