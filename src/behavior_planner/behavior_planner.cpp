#include "behavior_planner.h"

vector<VectorXd> behavior_planner::bestOption(){
    cout << "behavior_planner::bestOption" << endl;
    vector<trajectory_option*> trajectories;
    vector<future<void>> options;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        // TODO setup city planning code
        if(0 <= values->getVelocity(a)){
            trajectory_option* option = new trajectory_option();
            options.push_back(async(launch::deferred, [this, a, option]{ highwayPlanner->calculateS(a, option->score_s, option->s);})); // TODO make async before submission
            options.push_back(async(launch::deferred, [this, a, option]{ highwayPlanner->calculateD(a, option->score_d, option->d);})); // TODO make async before submission
            trajectories.push_back(option);
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

    options[0].get();
    options[1].get();
    VectorXd s  = trajectories[0]->s;
    VectorXd d  = trajectories[0]->d;
    double score_s = trajectories[0]->score_s;
    double score_d = trajectories[0]->score_d;
    cout << "lowest s score so far: " << score_s << "\t\tlowest d score so far: " << score_d << endl;
    for(short a = 1; a < trajectories.size(); a++){
        // cout << "a: " << a << endl;
        try{
            options[a*2].get();
            options[a*2+1].get();
            if(score_s +score_d > trajectories[a]->score_s  + trajectories[a]->score_d){
                score_s = trajectories[a]->score_s;
                s = trajectories[a]->s;

                score_d = trajectories[a]->score_d;
                d = trajectories[a]->d;
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

    // TODO change code to allow me to store the calculated acceleration 1 second into the future for s and d
    // suggest returning c_d and c_s as 3 element arrays instead of 6 element arrays
    // should be more efficient anyways
    // consider reusing vectors

    return {s, d};
}