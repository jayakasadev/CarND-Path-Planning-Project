#include "behavior_planner.h"

void behavior_planner::calculateHighwayS(short lane, double &score_s, VectorXd &s){
    cout << "behavior_planner::calculateHighwayS\t" << lane << endl;
    double sf = 0, sf_dot = 0, sf_dot_dot = 0;
    getSfVals(sf, sf_dot, lane);
    VectorXd c_s;
    for(short interval = 0; interval < spacing; interval++){
        double sf_interval = sf + interval;
        cout << "############# sf_inteval: " << sf_interval << endl;
        for(double time = 1.0; time <= 3.0; time+=refresh_rate) {
            // cout << "CALCULATE S" << endl;// generate matrices
            c_s = VectorXd::Zero(3);
            sharedCalc(time, car->getS(), car->getVelocityS(), car->getAccelerationS(), sf_interval, sf_dot, sf_dot_dot, c_s);
            // cout << c_s.transpose() << endl;

            // calculate cost
            // && velocity_f <= max_velocity_mps && velocity_f >= 0
            if(abs(c_s[0]) < max_jerk){
                cout << "\ttime: " << time;
                cout << "\tS -> jerk: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << endl;
                double acceleration_f = accelerationAvg(c_s, car->getAccelerationD());
                double jerk_f = jerkF(acceleration_f);
                cout << "\tjerk_f = " << jerk_f << endl;
                cout << "\tacceleration_f: " << acceleration_f << endl;

                if(abs(jerk_f) < max_jerk && abs(acceleration_f) < max_acceleration){
                    // cout << "B ";
                    double cost;
                    if (values->getBehavior(lane) == KEEP_VELOCITY) {
                        // cout << "C ";
                        double velocity_f = velocityF(c_s, car->getVelocityS(), car->getAccelerationS());
                        cout << "\tvelocity_f: " << velocity_f << endl;
                        double  diff = velocity_f - car->getVelocityS();
                        cost = costV(lane, time, diff, c_s);
                    } else {
                        // cout << "D ";
                        double position_f = positionF(c_s, time_period, car->getS(), car->getVelocityS(), car->getAccelerationS());
                        cout << "\tposition_f: " << position_f << endl;
                        double  diff = position_f - car->getS();
                        cost = costS(lane, time, diff, c_s);
                    }
                    cout << "\tcost: " << cost << endl;
                    // cout << "E ";
                    if (cost < score_s) {
                        // cout << "F " << endl;
                        score_s = cost;
                        VectorXd s_temp = VectorXd::Zero(6);
                        s_temp << car->getS(), car->getVelocityS(), car->getAccelerationS(), c_s[0], c_s[1], c_s[2];
                        s = s_temp;
                        // cout << "S: " << s.transpose() << endl;
                    }
                    // cout << "G ";
                }
            }
            // cout << endl;
        }
    }
}

void behavior_planner::calculateHighwayD(short lane, double &score_d, VectorXd &d){
    cout << "behavior_planner::calculateHighwayD\t" << lane << endl;
    double df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;
    cout << "\tdf: " << df;
    VectorXd c_d;
    for(double time = 1.0; time <= 3.0; time+=refresh_rate) {
        cout << "\ttime: " << time;
        c_d = VectorXd::Zero(3);
        // cout << "CALCULATE D" << endl;
        sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot, c_d);
        // cout << c_d.transpose() << endl;

        // calculate cost
        if (abs(c_d[0]) < max_jerk){
            cout << "\tD -> jerk: " << c_d[0] << " snap: " << c_d[1] << " crackle: " << c_d[2] << endl;
            // double velocity_f = velocityF(c_d, car->getVelocityD(), car->getAccelerationD());
            // double position_f = positionF(c_d, time, car->getD(), car->getVelocityD(), car->getAccelerationD());
            double acceleration_f = accelerationAvg(c_d, car->getAccelerationD());
            double jerk_f = jerkF(acceleration_f);
            cout << "\tjerk_f = " << jerk_f << endl;
            cout << "\tacceleration_f: " << acceleration_f << endl;
            // cout << "\tvelocity_f: " << velocity_f << endl;
            // cout << "\tposition_f: " << position_f << endl;
            if(abs(jerk_f) < max_jerk && abs(acceleration_f) < max_acceleration) {
                double diff = df - car->getD();
                double cost = costD(lane, time, diff, c_d);
                cout << "\tcost: " << cost << endl;
                if (cost < score_d) {
                    score_d = cost;
                    VectorXd d_temp = VectorXd::Zero(6);
                    d_temp << car->getD(), car->getVelocityD(), car->getAccelerationD(), c_d[0], c_d[1], c_d[2];
                    d = d_temp;
                    // cout << "D: " << d.transpose() << endl;
                }
            }
        }
    }
}

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
            options.push_back(async(launch::deferred, [this, a, option]{ calculateHighwayS(a, option->score_s, option->s);}));
            options.push_back(async(launch::deferred, [this, a, option]{ calculateHighwayD(a, option->score_d, option->d);}));
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