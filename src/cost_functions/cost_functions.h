//
// Created by jay on 12/28/17.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_H
#define PATH_PLANNING_COST_FUNCTIONS_H

#include <math.h>
#include <complex>

#include "../constants/constants.h"

class cost_functions {
private:
    double time_diff_cost(double time);

    double s_diff_cost(double s, double sf);

    double d_diff_cost(double d, double df);

    double efficieny_cost(double s);

public:
    cost_functions(){}
    ~cost_functions(){}

};


#endif //PATH_PLANNING_COST_FUNCTIONS_H
