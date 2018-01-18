#include "behavior_planner.h"

vector<trajectory> behavior_planner::plan(){
    // cout << "behavior_planner::bestOption" << endl;
    vector<future<void>> options;
    vector<planner *> planners;

    for(short a = 0; a < num_lanes; a++){
        // select behavior based on velocity
        // highway planning
        if((velocity_barrier * max_velocity_mps) <= car->getVelocityS()){
            cout << "highwayplanner" << endl;
            planners.push_back(highwayPlanner[a]);
            options.push_back(async(launch::deferred, [this, a]{ highwayPlanner[a]->calculate();}));
            // highwayPlanner[a].calculateS();
            // highwayPlanner[a].calculateD();
        } else {
            cout << "cityplanner" << endl;
            planners.push_back(cityPlanner[a]);
            options.push_back(async(launch::deferred, [this, a]{ cityPlanner[a]->calculate();}));
        }
    }

    // this part is synchronous
    // cannot compare without the completed computations
    // this part is only as slow as the slowest calculation

    // just make sure everything is done
    for(short a = 0; a < options.size(); a++){
        options[a].get();
    }

    planner * lowest = planners[0];

    return {};
}