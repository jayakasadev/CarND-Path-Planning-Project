#include "behavior_planner.h"

vector<trajectory_option> behavior_planner::plan(){
    cout << "behavior_planner::bestOption" << endl;
    vector<future<void>> options;
    vector<planner*> planners;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        if((velocity_barrier * max_velocity_mps) <= car->getVelocityS()){
            planners.push_back(&highwayPlanner[a]);
            options.push_back(async(launch::deferred, [this, a]{ highwayPlanner[a].calculateS();}));
            options.push_back(async(launch::deferred, [this, a]{ highwayPlanner[a].calculateD();}));
            // highwayPlanner[a].calculateS();
            // highwayPlanner[a].calculateD();
        } else {
            planners.push_back(&cityPlanner[a]);
            options.push_back(async(launch::deferred, [this, a]{ cityPlanner[a].calculate();}));
            // cityPlanner[a].calculate();
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

    planner * lowest = planners[0];

    // cout << "lowest s score so far: " << lowest->option_s.score << "\t\tlowest d score so far: " << lowest->option_d.score << endl;
    for(short a = 1; a < planners.size(); a++){
        // cout << "a: " << a << endl;
        if(lowest->option_s.score + lowest->option_d.score > (*planners[a]).option_s.score  + (*planners[a]).option_s.score){
            lowest = planners[a];
        }
        // cout << "lowest s score so far: " << lowest->option_s.score << "\t\tlowest d score so far: " << lowest->option_d.score << endl;
        // cout << "comparing it to this score: " << compare.score << endl;
    }

    // cout << "picked the best option" << endl;
    cout << "S: ";
    lowest->option_s.print();
    cout << "D: ";
    lowest->option_d.print();


    return {lowest->option_s, lowest->option_d};
}