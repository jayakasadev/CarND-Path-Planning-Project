#include "city_planner.h"

void city_planner::calculate(){
    // cout << "city_planner::calculate\tlane: " << lane << endl;
    // car->print();
    option_s.reset(car->getS(), car->getVelocityS(), car->getAccelerationS());
    option_d.reset(car->getD(), car->getVelocityD(), car->getAccelerationD());
    double sf = 0, sf_dot = 0, sf_dot_dot = 0;
    getSfVals(sf, sf_dot, lane);
    cout << "sf: " << sf << "\tsf_dot: " << sf_dot << "\tsf_dot_dot: " << sf_dot_dot << endl;

    double df = calculateTargetD(lane), df_dot = 0, df_dot_dot = 0;
    cout << "df: " << df << "\tdf_dot: " << df_dot << "\tdf_dot_dot: " << df_dot_dot << endl;

    VectorXd c_d;
    VectorXd c_s;

    for(double time = 1.0; time <= 3.0; time += refresh_rate) {

        c_s = sharedCalc(time, car->getS(), car->getVelocityS(), car->getAccelerationS(), sf, sf_dot, sf_dot_dot);
        c_d = sharedCalc(time, car->getD(), car->getVelocityD(), car->getAccelerationD(), df, df_dot, df_dot_dot);

        // calculate cost
        // && velocity_f <= max_velocity_mps && velocity_f >= 0
        if(abs(c_s[0]) < max_jerk && abs(c_d[0]) < max_jerk){
            // cout << "\ttime: " << time;
            // cout << "\tS -> jerk: " << c_s[0] << " snap: " << c_s[1] << " crackle: " << c_s[2] << endl;
            double acceleration_s = accelerationAvg(c_s, car->getAccelerationS());
            double acceleration_d = accelerationAvg(c_d, car->getAccelerationD());
            double jerk_s = jerkAvg(acceleration_s);
            double jerk_d = jerkAvg(acceleration_d);
            double velocity_s = velocityAvg(c_s, car->getVelocityS(), car->getAccelerationS());
            double velocity_d = velocityAvg(c_d, car->getVelocityD(), car->getAccelerationD());
            if(abs(jerk_s) < max_jerk && abs(acceleration_s) < max_acceleration && abs(jerk_d) < max_jerk
               && abs(acceleration_d) < max_acceleration && abs(velocity_s) < max_velocity * mph_to_mps
               && abs(velocity_d) < max_velocity * mph_to_mps){
                double diff_d = df - car->getD();
                double diff_s = sf - car->getS();
                double cost = costSD(time, diff_s, diff_d, c_d);
                cost = scoreFunction(cost);
                if (cost < option_d.score) {
                    // cout << "\tjerk_s = " << jerk_s << endl;
                    // cout << "\tacceleration_s: " << acceleration_s << endl;
                    // cout << "\tjerk_d = " << jerk_d << endl;
                    // cout << "\tacceleration_d: " << acceleration_d << endl;
                    // cout << "\tcost: " << cost << "\ttime:" << time << endl;
                    // cout << endl;
                    // cout << "\tD: " << c_d.transpose() << endl;
                    option_d.update(c_d, time, cost);
                    option_s.update(c_s, time, cost);
                    // cout << "\tacceleration: " << acceleration_f << "\tlane: " << lane << endl;
                    return; // stop at the first good solution, we are going slow enough already
                }
            }
        }
        // cout << endl;
    }
}