#include "highway_planner.h"

void highway_planner::calculateS(){
    // cout << "highway_planner::calculateS\tlane: " << lane << endl;
    option_s.reset(car->getS(), car->getVelocityS(), car->getAccelerationS());
    double sf = 0, sf_dot = 0, sf_dot_dot = 0;
    getSfVals(sf, sf_dot, lane);
    cout << "sf: " << sf << "\tsf_dot: " << sf_dot << "\tsf_dot_dot: " << sf_dot_dot << endl;
    VectorXd c_s;
    for(double time = 1.0; time <= 4.0; time += refresh_rate) {
        // cout << "CALCULATE S" << endl;// generate matrices

        c_s = sharedCalc(time, car->getS(), car->getVelocityS(), car->getAccelerationS(), sf, sf_dot, sf_dot_dot);
        // cout << c_s.transpose() << endl;

        // calculate cost
        // && velocity_f <= max_velocity_mps && velocity_f >= 0
        if(abs(c_s[0]) < max_jerk){
            // cout << "\ttime: " << time;
            // cout << "\tS -> jerk: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << endl;
            double acceleration_f = accelerationAvg(c_s, car->getAccelerationD());
            double jerk_avg = jerkAvg(acceleration_f);
            // cout << "\tjerk_f = " << jerk_f << endl;
            // cout << "\tacceleration_f: " << acceleration_f << endl;

            if(abs(jerk_avg) < max_jerk && abs(acceleration_f) < max_acceleration){
                // cout << "B ";
                double velocity_f = velocityF(c_s, time, car->getVelocityS(), car->getAccelerationS());
                double position_f = positionF(c_s, time, car->getS(), car->getVelocityS(), car->getAccelerationS());
                // cout << "\tvelocity_f: " << velocity_f << endl;
                // cout << "\tposition_f: " << position_f << endl;
                double cost;
                if (values->getBehavior(lane) == KEEP_VELOCITY) {
                    // cout << "C ";
                    double  diff = velocity_f - car->getVelocityS();
                    cost = costV(time, diff, c_s);
                } else {
                    // cout << "D ";
                    double  diff = position_f - car->getS();
                    cost = costS(time, diff, c_s);
                }
                // cost = scoreFunction(cost);
                // cout << "\tcost: " << cost << endl;
                // cout << "E ";
                if (cost < option_s.score) {
                    // cout << "\tcost: " << cost << "\ttime:" << time << endl;
                    // cout << "\tS: " << c_s.transpose() << endl;
                    option_s.update(c_s, time, cost);
                    // option_s.xf = positionF(c_s, time_period, car->getS(), car->getVelocityS(), car->getAccelerationS());
                    // option_s.xf_dot = velocityF(c_s, time_period, car->getVelocityS(), car->getAccelerationS());
                    // option_s.xf_dot_dot = accelerationF(c_s, time_period, car->getAccelerationS());
                    // cout << "\tposition: " << position_f << "\tlane: " << lane << endl;
                    // cout << "\tvelocity: " << velocity_f << "\tlane: " << lane << endl;
                    // cout << "\tacceleration: " << accelerationF(c_s, time_period, car->getAccelerationS()) << "\tlane: " << lane << endl;
                    // cout << endl;
                    // cout << "\toption_s.xf_dot_dot: " << option_s.xf_dot_dot << "\tlane: " << lane << endl;
                    // s_v = velocity_f;
                    // s_a = accelerationF(c_s, time_period, car->getAccelerationS());

                }
                // cout << "G ";
            }
        }
        // cout << endl;
    }
}

void highway_planner::calculateD(){
    // cout << "highway_planner::calculateD\t" << lane << endl;
    option_d.reset(car->getD(), car->getVelocityD(), car->getAccelerationD());
    double df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;
    cout << "df: " << df << "\tdf_dot: " << df_dot << "\tdf_dot_dot: " << df_dot_dot << endl;
    VectorXd c_d;
    for(double time = 1.0; time <= 4.0; time+=refresh_rate) {
        // c_d = VectorXd::Zero(3);
        // cout << "CALCULATE D" << endl;
        c_d = sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot);
        // cout << c_d.transpose() << endl;

        // calculate cost
        if(abs(c_d[0]) < max_jerk){
            // cout << "\ttime: " << time;
            // cout << "\tD -> jerk: " << c_d[0] << " snap: " << c_d[1] << " crackle: " << c_d[2] << endl;
            // double velocity_f = velocityF(c_d, time, car->getVelocityD(), car->getAccelerationD());
            // double position_f = positionF(c_d, time, car->getD(), car->getVelocityD(), car->getAccelerationD());
            double acceleration_f = accelerationAvg(c_d, car->getAccelerationD());
            double jerk_f = jerkAvg(acceleration_f);
            // cout << "\tjerk_f = " << jerk_f << endl;
            // cout << "\tacceleration_f: " << acceleration_f << endl;
            // cout << "\tvelocity_f: " << velocity_f << endl;
            // cout << "\tposition_f: " << position_f << endl;
            if(abs(jerk_f) < max_jerk && abs(acceleration_f) < max_acceleration) {
                double diff = df - car->getD();
                double cost = costD(time, diff, c_d);
                cost = scoreFunction(cost);
                // cout << "\tcost: " << cost << endl;
                if (cost < option_d.score) {
                    // cout << "\tcost: " << cost << "\ttime:" << time << endl;
                    // cout << "\tD: " << c_d.transpose() << endl;
                    option_d.update(c_d, time, cost);
                    // cout << "\tacceleration: " << acceleration_f << "\tlane: " << lane << endl;
                }
            }
        }
    }
}