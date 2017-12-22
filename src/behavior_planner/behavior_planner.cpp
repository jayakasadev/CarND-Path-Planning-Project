#include "behavior_planner.h"

void behavior_planner::bestOption(){
    cout << "behavior_planner::bestOption" << endl;
    vector<trajectory_option> trajectories;
    vector<future<void>> options;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        // TODO setup city planning code
        if(0 <= values->getVelocity(a)){
            trajectory_option curr_option;
            options.push_back(async(launch::deferred, [this, a, &curr_option]{ highwayPlanner->calculateS(a, curr_option.scoreS, curr_option.timeS, *curr_option.s);})); // TODO make async before submission
            options.push_back(async(launch::deferred, [this, a, &curr_option]{ highwayPlanner->calculateD(a, curr_option.scoreD, curr_option.timeD, *curr_option.d);})); // TODO make async before submission
            trajectories.push_back(curr_option);
        } else {
            // TODO implement city planning
            // do nothing for now
            // options.push_back(async(launch::async, [this, &a]{return this->cityPlanning(a);}));
        }
    }

    // cout << "finished calculations: " << options.size() << endl;

    // this part is synchronous
    // cannot compare without the completed computations
    // this part is only as slow as the slowest calculation

    // just make sure everything is done
    for(short a = 0; a < options.size(); a++){
        options[a].get();
    }

    trajectory_option best_option = trajectories[0];
    cout << "lowest s score so far: " << best_option.scoreS << "\t\tlowest d score so far: " << best_option.scoreD << endl;
    for(short a = 1; a < trajectories.size(); a++){
        // cout << "a: " << a << endl;
        try{
            if(best_option.scoreS + best_option.scoreS > trajectories[a].scoreS  + trajectories[a].scoreD){
                best_option = trajectories[a];
            }
            cout << "lowest s score so far: " << best_option.scoreS << "\t\tlowest d score so far: " << best_option.scoreS << endl;
            // cout << "comparing it to this score: " << compare.score << endl;
        } catch (future_error &e){
            cout << e.what() << endl;
        }
    }

    // cout << "picked the best option" << endl;
    cout << "S: " << best_option.s->transpose() << endl;
    cout << "D: " << best_option.d->transpose() << endl;

    // TODO change code to allow me to store the calculated acceleration 1 second into the future for s and d
    // suggest returning c_d and c_s as 3 element arrays instead of 6 element arrays
    // should be more efficient anyways
    // consider reusing vectors

    // delete option->s;
    // delete option->d;
    VectorXd temp_s = VectorXd::Zero(6);
    temp_s << car->getS(), car->getVelocityS(), car->getAccelerationS(), best_option.s[0], best_option.s[1], best_option.s[2];
    option->s = &temp_s;

    VectorXd temp_d = VectorXd::Zero(6);
    temp_d << car->getD(), car->getVelocityD(), car->getAccelerationD(), best_option.d[0], best_option.d[1], best_option.d[2];
    option->d = &temp_d;

    // option->scoreS = best_option->scoreS;
    // option->scoreD = best_option->scoreD;
    option->timeS = best_option.timeS;
    option->timeD = best_option.timeD;
}