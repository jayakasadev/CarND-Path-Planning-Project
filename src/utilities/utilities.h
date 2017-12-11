//
// Created by jay on 12/5/17.
//

#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

#include <math.h>
#include "../constants/constants.h"

// For converting back and forth between radians and degrees.
static inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline double calculateFutureS(double s){
    return fmod(s, max_s);
}

inline float calculateTargetD(int lane){
    return 4 * lane + 2;
}

inline short calculateLane(double d){
    short lane = (d / 4);
    if(lane < 0){
        return 0;
    } else if(lane >= num_lanes){
        return num_lanes - 1;
    }
    return lane;
}

inline double calculateYaw(double ref_y, double ref_y_prev, double ref_x, double ref_x_prev){ // returns in radians
    return atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
}

inline int getNumPoints(float time_period){
    return int(time_period / refresh_rate);
}

#endif //PATH_PLANNING_UTILITIES_H
