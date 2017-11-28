/**
 * Methods provided by udacity for the students
 */

#ifndef UTILTIES_H_
#define UTILTIES_H_

#include <math.h>
#include <vector>
#include <string>
#include "constants.h"

using std::vector;
using std::string;

string hasData(string s);

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y);

int NextWaypoint(double x, double y, double theta);

vector<double> getFrenet(double x, double y, double theta);

vector<double> getXY(double s, double d);

inline double getYaw(double ref_y, double ref_y_prev, double ref_x, double ref_x_prev);

inline int min_element(float arr[], int size);

#endif //UTILTIES_H_
