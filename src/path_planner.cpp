//
// Created by Friedrich Schweizer on 29.07.17.
//

#include "path_planner.h"

/**
 * Initialize PP
 */
PP::PP() {
    this->speed_limit = 21.0;

    this->car_x = 0.0;
    this->car_y = 0.0;
    this->car_s = 0.0;
    this->car_d = 0.0;
    this->car_yaw = 0.0;
    this->car_speed = 0.0;
    this->car_acceleration = 0.0;

    this->number_of_points = 49;
}

PP::~PP() {}

void PP::init_map(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy){

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

    for (int i=0; i<10000; i++)
    {
        double t = (double)i / (double)10000;
        this->maps_x.push_back(waypoint_spline_x(t));
        this->maps_y.push_back(waypoint_spline_y(t));
        this->maps_s.push_back(waypoint_spline_s(t));
        this->maps_dx.push_back(waypoint_spline_dx(t));
        this->maps_dy.push_back(waypoint_spline_dy(t));
    }
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
    //vector<double> frenet_pos = getFrenet(initial_state[0], initial_state[3], atan2(initial_state[4],initial_state[1]), this->maps_x, this->maps_y);

    this->goal_s = initial_state[0] + this->behaviour_speed;

    this->goal_d = this->behaviour_lane_id * 4.0 + 2.0;

    vector<double> final_pos = getXY(this->goal_s, this->goal_d , this->maps_s, this->maps_x, this->maps_y);
    this->goal_x = final_pos[0];
    this->goal_y = final_pos[1];
    int nwp = NextWaypoint(this->goal_x, this->goal_y, this->car_yaw, this->maps_x, this->maps_y);
    double theta = atan2(this->maps_dy[nwp], this->maps_dx[nwp]) + (pi()/2.0);
    if(theta >= (2.0*pi())){theta -= 2*pi();}
    this->goal_yaw = theta;
    this->goal_speed = this->speed_limit;

    cout << "X_Diff: " << initial_state[0] << " " << this->goal_x << endl;
    cout << rad2deg(this->car_yaw) << " " << rad2deg(theta) << endl;
}

void PP::predict(json sensor_fusion) {
    for(int i = 0; i < sensor_fusion.size(); i++){

        //cout << sensor_fusion[i][1].get<double>() << endl;

        this->predictions.erase(this->predictions.begin(), this->predictions.end());

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double next_x = sensor_fusion[i][1].get<double>() + vx*1.0;
        double next_y = sensor_fusion[i][2].get<double>() + vy*1.0;
        double theta = atan2(vy,vx);

        vector<double>next_frenet = getFrenet(next_x, next_y, theta, this->maps_x, this->maps_y);
        int lane_id = (int)(((next_frenet[1] - 2.0) / 4.0) + 0.5);
        //cout << lane_id << endl;

        vector<double> prediction = {next_x, next_y, vx, vy, next_frenet[0], next_frenet[1]};

        this->predictions[lane_id].push_back(prediction);



        //cout << sensor_fusion[i][5].get<double>() << " " << next_frenet[0] << endl;
    }
}

void PP::behaviour_planning(){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */

    double max_s = 6945.554;

    int next_vehicle_in_lane_id;
    double next_vehicle_in_lane_s = max_s;

    for(int i=0; i<this->predictions[this->car_lane_id].size(); i++){
        double delta_d = this->predictions[this->car_lane_id][i][5]-this->car_d;
        double delta_s = fmod((this->predictions[this->car_lane_id][i][4]-this->car_s+max_s),max_s);

        if(abs(delta_d) < 2.0){
            if(delta_s<next_vehicle_in_lane_s){
                next_vehicle_in_lane_id = i;
                next_vehicle_in_lane_s = delta_s;
            }
        }
    }

    this->behaviour_speed += 0.2;
    if(this->behaviour_speed > this->speed_limit){
        this->behaviour_speed = this->speed_limit;
    }
    this->behaviour_lane_id = this->car_lane_id;

    if(next_vehicle_in_lane_s < 35.0){
        this->behaviour_lane_id += 1;
    }else{
        this->behaviour_lane_id -= 1;
        if(this->behaviour_lane_id < 0){
            this->behaviour_lane_id = 0;
        }
    }




}

void PP::create_trajectory(json previous_path_x, json previous_path_y){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */

    int current_path_point = this->number_of_points - previous_path_x.size();

    if(current_path_point>10){
        cout << "NEW TRAJECTORY after: " << current_path_point << " Points" << endl;

        double dt = (1.0 / 50.0);

        if(this->trajectory.size() == 0){
            this->trajectory.push_back({this->car_s, this->car_speed, 0.0, this->car_d, 0.0, 0.0});
        }else{
            this->trajectory.erase(this->trajectory.begin(), this->trajectory.begin()+current_path_point);
        }

        cout << "Tracjectory Size: " << this->trajectory.size() << endl;

        this->define_goal_state(this->trajectory.back());

        vector<double> initial_s_state = {this->trajectory.back()[0], this->trajectory.back()[1], this->trajectory.back()[2]};
        vector<double> final_s_state = {this->goal_s, this->behaviour_speed, 0.0};

        vector<double> initial_d_state = {this->trajectory.back()[3], this->trajectory.back()[4], this->trajectory.back()[5]};
        vector<double> final_d_state = {this->goal_d, 0.0, 0.0};

        cout << "Init Speed: " << this->trajectory.back()[1] << " Goal Speed: " << this->goal_speed << endl;

        vector<double> s_coeffs = this->JMT(initial_s_state, final_s_state, 1.0);
        vector<double> d_coeffs = this->JMT(initial_d_state, final_d_state, 1.0);


        for(int i=1; i<=current_path_point; i++){

            double t = dt * (double)i;

            double s_t = 0.0;
            for(int coeff=0; coeff<6; coeff++){
                s_t += s_coeffs[coeff] * pow(t, (double)coeff);
            }

            double d_s_t = 0.0;
            for(int coeff=1; coeff<6; coeff++){
                d_s_t += s_coeffs[coeff] * (double)coeff * pow(t, (double)(coeff-1));
            }

            double dd_s_t = 0.0;
            for(int coeff=2; coeff<6; coeff++){
                dd_s_t += s_coeffs[coeff] * ((double)coeff * (double)(coeff-1)) * pow(t, (double)(coeff-2));
            }

            double d_t = 0.0;
            for(int coeff=0; coeff<6; coeff++){
                d_t += d_coeffs[coeff] * pow(t, (double)coeff);
            }

            double d_d_t = 0.0;
            for(int coeff=1; coeff<6; coeff++){
                d_d_t += d_coeffs[coeff] * (double)coeff * pow(t, (double)(coeff-1));
            }

            double dd_d_t = 0.0;
            for(int coeff=2; coeff<6; coeff++){
                dd_d_t += d_coeffs[coeff] * ((double)coeff * (double)(coeff-1)) * pow(t, (double)(coeff-2));
            }

            this->trajectory.push_back({s_t, d_s_t, dd_s_t, d_t, d_d_t, dd_d_t});
        }
        this->number_of_points = this->trajectory.size();
        cout << "Tracjectory Size to drive: " << this->trajectory.size() << endl;


        vector<double> next_x_vals;
        vector<double> next_y_vals;

        // The max s value before wrapping around the track back to 0
        double max_s = 6945.554;


        for(int i=0; i<this->number_of_points; i++){
            double s_temp = fmod(this->trajectory[i][0],max_s);
            vector<double> next_point = getXY(s_temp, this->trajectory[i][3], this->maps_s, this->maps_x, this->maps_y);
            next_x_vals.push_back(next_point[0]);
            next_y_vals.push_back(next_point[1]);
        }
        this->next_vals.erase(this->next_vals.begin(), this->next_vals.end());
        this->next_vals.push_back(next_x_vals);
        this->next_vals.push_back(next_y_vals);

    }


}


vector< double> PP::JMT(vector< double> start, vector <double> end, double T){
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

