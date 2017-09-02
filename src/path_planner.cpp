//
// Created by Friedrich Schweizer on 29.07.17.
//

#include "path_planner.h"

/**
 * Initialize PP
 */
PP::PP() {
    this->speed_limit = 49.3; //mph
    this->mac_acc = .55;

    this->car_x = 0.0;
    this->car_y = 0.0;
    this->car_s = 0.0;
    this->car_d = 0.0;
    this->car_yaw = 0.0;
    this->car_speed = 0.0;

    this->number_of_points = 50;
    this->car_lane_id = 1;
    this->goal_lane = 1;

    this->goal_speed = 0.0;

}

PP::~PP() {}

void PP::init_map(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy){
    /** Initialize the map in the PathPlanner Object **/
    this->maps_x = maps_x;
    this->maps_y = maps_y;
    this->maps_s = maps_s;
    this->maps_dx = maps_dx;
    this->maps_dy = maps_dy;

    /*
    // Spline interpolate the map waypoints
    vector<double> waypoint_spline_t = {};
    int map_waypoints_size = maps_x.size();
    for (int i=0; i<map_waypoints_size; i++)
    {
       double t = (double)i / (double)map_waypoints_size;
       waypoint_spline_t.push_back(t);
    }


    tk::spline waypoint_spline_x;
    waypoint_spline_x.set_points(waypoint_spline_t, maps_x);
    tk::spline waypoint_spline_y;
    waypoint_spline_y.set_points(waypoint_spline_t, maps_y);
    tk::spline waypoint_spline_s;
    waypoint_spline_s.set_points(waypoint_spline_t, maps_s);
    tk::spline waypoint_spline_dx;
    waypoint_spline_dx.set_points(waypoint_spline_t, maps_dx);
    tk::spline waypoint_spline_dy;
    waypoint_spline_dy.set_points(waypoint_spline_t, maps_dy);

    for (int i=0; i<1000; i++)
    {
       double t = (double)i / (double)1000;
       this->maps_x.push_back(waypoint_spline_x(t));
       this->maps_y.push_back(waypoint_spline_y(t));
       this->maps_s.push_back(waypoint_spline_s(t));
       this->maps_dx.push_back(waypoint_spline_dx(t));
       this->maps_dy.push_back(waypoint_spline_dy(t));
    }
    */
}

void PP::update_car_status(vector<double> car_status){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */
    this->car_x = car_status[0];
    this->car_y = car_status[1];
    this->car_s = car_status[2];
    this->car_d = car_status[3];
    this->car_yaw = deg2rad(car_status[4]);
    this->car_speed = car_status[5] * 0.44704;
    this->car_lane_id = (int)(((car_status[3] - 2.0) / 4.0) + 0.5);
}

void PP::define_goal_state(vector<double> initial_state){
    /** CURRENTLY NOT USE **/
    this->goal_s = initial_state[0] + this->behaviour_speed;

    this->goal_d = this->behaviour_lane_id * 4.0 + 2.0;

    vector<double> final_pos = getXY(this->goal_s, this->goal_d , this->maps_s, this->maps_x, this->maps_y);
    this->goal_x = final_pos[0];
    this->goal_y = final_pos[1];
    int nwp = NextWaypoint(this->goal_x, this->goal_y, this->car_yaw, this->maps_x, this->maps_y);
    double theta = atan2(this->maps_dy[nwp], this->maps_dx[nwp]) + (pi()/2.0);
    if(theta >= (2.0*pi())){theta -= 2*pi();}
    this->goal_speed = this->speed_limit;

    cout << "X_Diff: " << initial_state[0] << " " << this->goal_x << endl;
    cout << rad2deg(this->car_yaw) << " " << rad2deg(theta) << endl;
}

void PP::predict(json sensor_fusion, int prev_size) {
    /** Predict all other vehicles detected by SensorFusion **/
    this->predictions[0].erase(this->predictions[0].begin(), this->predictions[0].end());
    this->predictions[1].erase(this->predictions[1].begin(), this->predictions[1].end());
    this->predictions[2].erase(this->predictions[2].begin(), this->predictions[2].end());

    for(int i = 0; i < sensor_fusion.size(); i++){

        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];

        double speed = sqrt((vx*vx)+(vy*vy));
        double next_x = x + (vx * (double)prev_size * 0.02);
        double next_y = y + (vy * (double)prev_size * 0.02);
        double next_s = s + speed * 1.0;
        //double theta = atan2(vy,vx);

        int lane_id = (int)(((d - 2.0) / 4.0) + 0.5);
        //cout << lane_id << endl;

        vector<double> prediction = {next_x, next_y, next_s, speed, vx, vy};

        this->predictions[lane_id].push_back(prediction);

        //cout << sensor_fusion[i][5].get<double>() << " " << next_frenet[0] << endl;
    }
}

void PP::behaviour_planning(double end_path_s, double end_path_d, int prev_size){
    /** Plan the behaviour of the EGO-vehicle **/

    if(prev_size > 0){
        this->car_s = end_path_s;
        this->car_d = end_path_d;
        this->car_lane_id = (int)(((this->car_d - 2.0) / 4.0) + 0.5);
    }

    vector<double> front_rear_vehicles;
    front_rear_vehicles = find_next_vehicles_in_lane(this->goal_lane);

    if(this->goal_lane == this->car_lane_id) {
        if (front_rear_vehicles[0] < (this->car_speed + 10.0)) {
            int left_lane = this->car_lane_id - 1;
            int right_lane = this->car_lane_id + 1;
            //int left_left_lane = this->car_lane_id -2;
            //int right_right_lane = this->car_lane_id +2;

            bool lane_change = false;
            vector<double> lane_change_front_rear_vehicles;

            if (!lane_change && left_lane >= 0) {
                lane_change_front_rear_vehicles = find_next_vehicles_in_lane(left_lane);
                if ((lane_change_front_rear_vehicles[0] > (this->car_speed + 10.0)) &&
                    (lane_change_front_rear_vehicles[1] < -(10.0))) {
                    this->goal_lane = this->car_lane_id - 1;
                    lane_change = true;
                }
            }

            if (!lane_change && right_lane < 3) {
                lane_change_front_rear_vehicles = find_next_vehicles_in_lane(right_lane);
                if ((lane_change_front_rear_vehicles[0] > (this->car_speed + 10.0)) &&
                    (lane_change_front_rear_vehicles[1] < -(10.0))) {
                    this->goal_lane = this->car_lane_id + 1;
                    lane_change = true;
                }
            }

            if (!lane_change) {
                this->goal_speed -= mac_acc;
            }

        } else if (this->goal_speed < this->speed_limit) {
            this->goal_speed += mac_acc;
        }
    }

}

void PP::create_trajectory(int prev_size){
    /** Create a smooth trajectory in regard to the given criteria **/

    int current_path_point = this->number_of_points - prev_size;
    cout << "NEW TRAJECTORY after: " << current_path_point << " Points" << endl;

    vector<double> waypoints_x;
    vector<double> waypoints_y;

    double last_ref_x;
    double last_ref_y;
    double ref_x;
    double ref_y;
    double ref_yaw;

    if(current_path_point > 48){
        last_ref_x = this->car_x - cos(this->car_yaw);
        last_ref_y = this->car_y - sin(this->car_yaw);

        ref_x = this->car_x;
        ref_y = this->car_y;
        ref_yaw = this->car_yaw;

    }else{
        last_ref_x = this->trajectory_x[50-2];
        last_ref_y = this->trajectory_y[50-2];

        ref_x = this->trajectory_x[50-1];
        ref_y = this->trajectory_y[50-1];
        ref_yaw = atan2(ref_y-last_ref_y,ref_x-last_ref_x);
    }

    waypoints_x.push_back(last_ref_x);
    waypoints_y.push_back(last_ref_y);

    waypoints_x.push_back(ref_x);
    waypoints_y.push_back(ref_y);

    vector<double> next_waypoint_30 = getXY(this->car_s+30, 2+4*this->goal_lane, this->maps_s, this->maps_x, this->maps_y);
    waypoints_x.push_back(next_waypoint_30[0]);
    waypoints_y.push_back(next_waypoint_30[1]);

    vector<double> next_waypoint_60 = getXY(this->car_s+60, 2+4*this->goal_lane, this->maps_s, this->maps_x, this->maps_y);
    waypoints_x.push_back(next_waypoint_60[0]);
    waypoints_y.push_back(next_waypoint_60[1]);

    vector<double> next_waypoint_90 = getXY(this->car_s+90, 2+4*this->goal_lane, this->maps_s, this->maps_x, this->maps_y);
    waypoints_x.push_back(next_waypoint_90[0]);
    waypoints_y.push_back(next_waypoint_90[1]);


    for (int i=0;i<waypoints_x.size();i++){
        double shift_x = waypoints_x[i] - ref_x;
        double shift_y = waypoints_y[i] - ref_y;

        waypoints_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        waypoints_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }

    tk::spline s;
    s.set_points(waypoints_x,waypoints_y);

    if(this->trajectory_x.size() > 0){
        this->trajectory_x.erase(this->trajectory_x.begin(), this->trajectory_x.begin()+current_path_point);
        this->trajectory_y.erase(this->trajectory_y.begin(), this->trajectory_y.begin()+current_path_point);
    }

    double horizon_x = 30.0;
    double horizon_y = s(horizon_x);
    double horizon_dist = sqrt((horizon_x*horizon_x)+(horizon_y*horizon_y));

    double x_counter = 0.0;
    double delta_x = horizon_x * (0.02 * this->goal_speed / 2.24) / horizon_dist;

    for (int i=1;i<=current_path_point;i++){
        x_counter = x_counter + delta_x;
        double y_counter = s(x_counter);

        double next_x = (x_counter * cos(ref_yaw) - y_counter * sin(ref_yaw));
        double next_y = (x_counter * sin(ref_yaw) + y_counter * cos(ref_yaw));

        next_x += ref_x;
        next_y += ref_y;

        this->trajectory_x.push_back(next_x);
        this->trajectory_y.push_back(next_y);

    }

}


vector<double> PP::find_next_vehicles_in_lane(int lane_id){
    double max_s = 6945.554;

    double front_vehicle_in_lane_s = max_s;
    double back_vehicle_in_lane_s = -max_s;
    for(int i=0; i<this->predictions[lane_id].size(); i++){
        double delta_s = (this->predictions[lane_id][i][2]-this->car_s);
        if(delta_s < -max_s/2.0){
            delta_s += max_s;
        }

        if((delta_s>0.0)&&(delta_s < front_vehicle_in_lane_s)){
            front_vehicle_in_lane_s = delta_s;
        }else if((delta_s<0.0)&&(delta_s > back_vehicle_in_lane_s)){
            back_vehicle_in_lane_s = delta_s;
        }
    }
    return {front_vehicle_in_lane_s, back_vehicle_in_lane_s};
}


vector< double> PP::JMT(vector< double> start, vector <double> end, double T){
    /** CURRENTLY NOT USE **/
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
            3*T*T, 4*T*T*T,5*T*T*T*T,
            6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3, 1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
            end[1]-(start[1]+start[2]*T),
            end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    return result;

}

