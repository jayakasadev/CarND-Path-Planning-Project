#include "highway_planner.h"

void highway_planner::calculateS(short lane, double &score_s, double &time_s, VectorXd &s){
    cout << "behavior_planner::calculateHighwayS\t" << lane << endl;
    double sf = 0, sf_dot = 0, sf_dot_dot = 0;
    getSfVals(sf, sf_dot, lane);
    VectorXd c_s = VectorXd::Zero(3);
    for(short interval = 0; interval < spacing; interval++){
        double sf_interval = sf + interval;
        cout << "############# sf_inteval: " << sf_interval << endl;
        for(double time = 1.0; time <= 3.0; time += refresh_rate) {
            // cout << "CALCULATE S" << endl;// generate matrices

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
                    cout << "B ";
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
                    cout << "E ";
                    if (cost < score_s) {
                        cout << "F " << endl;
                        score_s = cost;
                        time_s = time;
                        VectorXd s_temp = VectorXd::Zero(3);
                        s_temp << c_s[0], c_s[1], c_s[2];
                        s = s_temp;
                        // cout << "S: " << s.transpose() << endl;
                    }
                    cout << "G ";
                }
            }
            cout << endl;
        }
    }
}

void highway_planner::calculateD(short lane, double &score_d, double &time_d, VectorXd &d){
    cout << "behavior_planner::calculateHighwayD\t" << lane << endl;
    double df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;
    cout << "\tdf: " << df;
    VectorXd c_d = VectorXd::Zero(3);
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
                    cout << "D: " << d.transpose() << endl;
                    score_d = cost;
                    time_d = time;
                    VectorXd d_temp = VectorXd::Zero(6);
                    d_temp << c_d[0], c_d[1], c_d[2];
                    d = d_temp;

                }
            }
        }
    }
}