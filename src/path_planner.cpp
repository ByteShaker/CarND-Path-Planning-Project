//
// Created by Friedrich Schweizer on 29.07.17.
//

#include "path_planner.h"

/**
 * Initialize PP
 */
PP::PP() {
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
    this->maps_s = maps_s;
    this->maps_x = maps_x;
    this->maps_y = maps_y;
    this->maps_dx = maps_dx;
    this->maps_dy = maps_dy;
}

void PP::update_car_status(vector<double> car_status){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */
    this->car_x = car_status[0];
    this->car_y = car_status[1];
    this->car_s = car_status[2];
    this->car_d = car_status[3];
    this->car_yaw = deg2rad(car_status[4]);
    this->car_speed = car_status[5] * 0.44704;
}

void PP::define_goal_state(vector<double> initial_state, int current_path_point){
    vector<double> frenet_pos = getFrenet(initial_state[0], initial_state[3], atan2(initial_state[4],initial_state[1]), this->maps_x, this->maps_y);

    this->goal_s = frenet_pos[0] + ((double)current_path_point*0.02*20.0);
    this->goal_d = 6.0;
    vector<double> final_pos = getXY(this->goal_s, this->goal_d , this->maps_s, this->maps_x, this->maps_y);
    this->goal_x = final_pos[0];
    this->goal_y = final_pos[1];
    int nwp = NextWaypoint(this->goal_x, this->goal_y, this->car_yaw, this->maps_x, this->maps_y);
    double theta = atan2(this->maps_dy[nwp], this->maps_dx[nwp]) + (pi()/2.0);
    if(theta >= (2.0*pi())){theta -= 2*pi();}
    this->goal_yaw = theta;
    this->goal_speed = 20.0;

    cout << "X_Diff: " << initial_state[0] << " " << this->goal_x << endl;
    cout << rad2deg(this->car_yaw) << " " << rad2deg(theta) << endl;
}

void PP::predict(json sensor_fusion) {
    for(int i = 0; i < sensor_fusion.size(); i++){

        //cout << sensor_fusion[i][1].get<double>() << endl;

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double next_x = sensor_fusion[i][1].get<double>() + vx*1.0;
        double next_y = sensor_fusion[i][2].get<double>() + vy*1.0;
        double theta = atan2(vy,vx);

        vector<double>next_frenet = getFrenet(next_x, next_y, theta, this->maps_x, this->maps_y);

        vector<double> prediction = {next_x, next_y, vx, vy, next_frenet[0], next_frenet[1]};

        this->predictions[sensor_fusion[i][0].get<int>()] = prediction;

        //cout << sensor_fusion[i][5].get<double>() << " " << next_frenet[0] << endl;
    }
}

void PP::behaviour_planning(){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */

    this->current_behaviour = "KL";

}

void PP::create_trajectory(json previous_path_x, json previous_path_y){
    /** vector<double> car_status = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */

    int current_path_point = this->number_of_points - previous_path_x.size();

    if(current_path_point>10){
        cout << "NEW TRAJECTORY after: " << current_path_point << " Points" << endl;

        double dt = (1.0 / 50.0);

        if(this->trajectory.size() == 0){
            this->trajectory.push_back({this->car_x, this->car_speed * cos(this->car_yaw), 0.0, this->car_y, this->car_speed * sin(this->car_yaw), 0.0});
        }else{
            this->trajectory.erase(this->trajectory.begin(), this->trajectory.begin()+current_path_point);
        }

        cout << "Tracjectory Size: " << this->trajectory.size() << endl;

        this->define_goal_state(this->trajectory.back(), current_path_point);

        vector<double> initial_x_state = {this->trajectory.back()[0], this->trajectory.back()[1], this->trajectory.back()[2]};
        vector<double> final_x_state = {this->goal_x, this->goal_speed * cos(this->goal_yaw), 0.0};

        vector<double> initial_y_state = {this->trajectory.back()[3], this->trajectory.back()[4], this->trajectory.back()[5]};
        vector<double> final_y_state = {this->goal_y, this->goal_speed * sin(this->goal_yaw), 0.0};

        cout << "Init Speed: " << pow(this->trajectory.back()[1],2.0)+pow(this->trajectory.back()[4],2.0) << " Goal Speed: " << pow(this->goal_speed * sin(this->goal_yaw),2.0)+pow(this->goal_speed * cos(this->goal_yaw),2.0) << endl;

        vector<double> x_coeffs = this->JMT(initial_x_state, final_x_state, (double)current_path_point*dt);
        vector<double> y_coeffs = this->JMT(initial_y_state, final_y_state, (double)current_path_point*dt);


        for(int i=1; i<=current_path_point; i++){

            double t = dt * (double)i;

            double x_t = 0.0;
            for(int coeff=0; coeff<6; coeff++){
                x_t += x_coeffs[coeff] * pow(t, (double)coeff);
            }

            double d_x_t = 0.0;
            for(int coeff=1; coeff<6; coeff++){
                d_x_t += x_coeffs[coeff] * pow(t, (double)(coeff-1));
            }

            double dd_x_t = 0.0;
            for(int coeff=2; coeff<6; coeff++){
                dd_x_t += x_coeffs[coeff] * pow(t, (double)(coeff-2));
            }

            double y_t = 0.0;
            for(int coeff=0; coeff<6; coeff++){
                y_t += y_coeffs[coeff] * pow(t, (double)coeff);
            }

            double d_y_t = 0.0;
            for(int coeff=1; coeff<6; coeff++){
                d_y_t += y_coeffs[coeff] * pow(t, (double)(coeff-1));
            }

            double dd_y_t = 0.0;
            for(int coeff=2; coeff<6; coeff++){
                dd_y_t += y_coeffs[coeff] * pow(t, (double)(coeff-2));
            }

            this->trajectory.push_back({x_t, d_x_t, dd_x_t, y_t, d_y_t, dd_y_t});
        }
        this->number_of_points = this->trajectory.size();
        cout << "Tracjectory Size to drive: " << this->trajectory.size() << endl;
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

