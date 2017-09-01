//
// Created by Friedrich Schweizer on 29.07.17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "helper_functions.h"
#include "json.hpp"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

using namespace std;

class PP {
public:

    int number_of_points;
    double speed_limit;

    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;
    vector<double> maps_dx;
    vector<double> maps_dy;

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    double car_acceleration;
    int car_lane_id;

    double goal_x;
    double goal_y;
    double goal_s;
    double goal_d;
    double goal_yaw;
    double goal_speed;
    double goal_acceleration;

    vector<string> states = {"KL","LCL","LCR","PLCL","PLCR"};

    vector<vector<vector<double>>> predictions = {{},{},{}};
    double behaviour_speed = 2.0;
    int behaviour_lane_id;

    vector<vector<double>> trajectory;
    vector<vector<double>> next_vals;

    /**
     * Constructor
     */
     PP();

    /**
     * Destructor
     */
     virtual ~PP();

    void init_map(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy);

    vector<double> JMT(vector< double> start, vector <double> end, double T);

    void update_car_status(vector<double> car_status);
    void define_goal_state(vector<double> initial_state);

    /** Predict Movement of other cars */
    void predict(json sensor_fusion);

    /** Plan Behaviour of own car*/
    void behaviour_planning();

    /** Create Trajectory of own car*/
    void create_trajectory(json previous_path_x, json previous_path_y);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
