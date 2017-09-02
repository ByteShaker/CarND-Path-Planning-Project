[video1]: https://youtu.be/pGVMAqtwk_s

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Solution of Friedrich Schweizer

##Project Overview

### In [THIS VIDEO](https://youtu.be/pGVMAqtwk_s) you will find the following criteria solved:
 
 + The Car is able to drive at least 4.32 miles without incident.
 
 + The car drives according to the speed limit.
 
 + Max Acceleration and Jerk are not Exceeded.
 
 + Car does not have collisions.
 
 + The car stays in its lane, except for the time between changing lanes.
 
 + The car is able to change lanes.
 
### General Approach

To solve this project I broke down the main problems into 3 parts:

1. Predict the behaviour of all other vehicles detected by Sensor Fusion

2. Plan the behaviour of the EGO-Vehicle

3. Generate a smooth trajectory based on the given criteria

Following I will describe the main points of my solution.

## Prediction

The track does have 3 lanes. For every detected vehicle I calculated the current lane ID. Also I measured the given leftover trajectory to know how long the new AddOn will be. Based on that time I predicted the future position of the other cars.
I did this by measuring their speed and calculating their current s value. Based on the Lane ID I created one vector for all vehicles per lane.

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
        }
    }

Those predictions are used in the following functions to calculate a valid behaviour for the EGO-vehicle.

## Behaviour Planning

The behaviour planner is always based on the last known position of the last given trajectory. 
Thats the same time as the predicton function predicted the other vehicles.

To start the Planning the algorithm finds the closest vehicles in the same lane as the EGO-vehicle. On in front of our car and one in the back.
If there is no car in front of us closer as a calculated distance based on our own speed the car will speed up to the speed limit. 
Regarding to the max acceleration.

If there is a slower car in front of us and to close we detect if it is save to change to a neighboured lane.
Therefore we look for the closest cars in those lanes, again front and rear.
In regard to german traffic I prefer left lanes. So if both neighboured lanes are available I make a left change.

Given the situation, that it is not save to change lanes anyhow. The car slowes down to avoid a crash with the car in front.

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

After this planner, the vehicle has a given goal lane and speed for its current situation.

## Trajectory Generation

Given my future goal-lane and -speed the algorithm creates a smooth trajectory, which the car can follow.
Therefore I take the last 2 points of my current trajectory and add 3 more points in further distances (30, 60, 90) and on the goal lane.
Those points are used to avoid sudden changes in ego-yaw and to create a smooth path without jerk.
Therefore I use the given Spline library.

Then I break down the calculated spline into pieces, that follow the rules of my current goal speed.
Based on those smaller pieces and the rest of the last created trajectory I create the new trajectory by adding new points to the end.



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

   
## Reflection

Since I was on vacation for more than 1 month this time, I only focused to meet the minimum rubric. 
The behaviour planner is pretty basic, but it does make valid decisions and drives savely over the loop.
Also the achieved speed is close to the speed limit and everything is smooth.

Nevertheless I still do see some room for perfection:
* Sometimes cars do change lanes unexpectedly. Therefore the prediction function could also try to predict lane changes of given cars.

* The behaviour planner could make the decisions based on more vehicles than only the closest ones. This could lead to a trajectory, that also plans the speed of the cars in front and makes better decisions on the long run.

* Also looking for explicit gaps, when the car is locked in and only can slow down. Could speed up lane changes, when needed.


Altogether I am pretty happy with my project solution. I am looking forward to add more refinement after the term is over.
   
