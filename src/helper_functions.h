//
// Created by Friedrich Schweizer on 29.07.17.
//

#ifndef PATH_PLANNING_HELPER_FUNCTIONS_H
#define PATH_PLANNING_HELPER_FUNCTIONS_H

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

double deg2rad(double x);
double rad2deg(double x);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

#endif //PATH_PLANNING_HELPER_FUNCTIONS_H
