#include "cost_functions.h"

double cost_functions::time_diff_cost(double time){
    return log(abs(time - time_period)) / time_period;
}

double cost_functions::s_diff_cost(double s, double sf){

}

double cost_functions::d_diff_cost(double d, double df){

}

double cost_functions::efficieny_cost(double s){

}