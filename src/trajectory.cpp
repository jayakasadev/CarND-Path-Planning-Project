//
// Created by jay on 11/15/17.
//

#include "trajectory.h"

void trajectory::generate(int prev_size, double car_x, double car_y, double car_yaw,
                          vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s,
                          vector<double> map_waypoints_x, vector<double> map_waypoints_y, double car_s, double speed, int lane){
    // make sure the next_x_vals and next_y_vals are empty
    next_x_vals.clear();
    next_y_vals.clear();

    /*
     * Create a list of widely spaced (x, y) waypoints, evenly spaced a 30 meters
     * interpolate these points with a spline and fill in with more points that control speed
     */
    vector<double> ptsx;
    vector<double> ptsy;

    /*
     * reference x,y, and yaw states
     * either reference the start point as where the car is or at the previous path's end point
     */
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // if previous size is almost empty, use the car as starting reference
    if(prev_size < 2){
        // cout << "2 or fewer points previously" << endl;
        // use two points that make the path tangent to the angle of the car
        // look where the car is at, and go backwards in time based on its angle
        // generate two points to make sure that path is tangent
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);

        // cout << "prev_car_x: " << prev_car_x << " prev_car_y: " << prev_car_y << endl;
        // cout << "car_x: " << car_x << " car_y: " << car_y << endl;

    } else {
        // cout << "more than 2 points previously" << endl;
        // look at the last couple points in the previous path that the car was following and then
        // calculate what angle the car was heading in using those last couple of points
        // then push these points onto a list of previous points

        // use the previous path's endpoint as a starting reference
        // Redefine reference state as previous path end point
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        // make sure the path is tangent to the the angle of the car by using the last two points in
        // the previous path
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use the two points that make up the path tangent to the previous path's end point
        ptsx.push_back(car_x);
        ptsx.push_back(ref_x_prev);

        ptsy.push_back(car_y);
        ptsy.push_back(ref_y_prev);

        // cout << "ref_x_prev: " << ref_x_prev << " ref_y_prev: " << ref_y_prev << endl;
        // cout << "car_x: " << car_x << " car_y: " << car_y << endl;
    }

    // at this point we have our starting reference points

    // In frenet add 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car_s + spacing, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 30m
    vector<double> next_wp1 = getXY(car_s + spacing * 2, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 60m
    vector<double> next_wp2 = getXY(car_s + spacing * 3, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 90m

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    /*
    for(int a = 0; a < ptsx.size(); a++){
        cout << "ptsx: " << ptsx[a] << " ptsy: " << ptsy[a] << endl;
    }
     */

    // cout << next_wp2[0] << " " << next_wp2[1] << endl;

    // now have 5 points on the points lists

    // transform to car's local coordinates or to the car's point of view
    // car is the origin at (0, 0)
    // we shift the points so that the car or the last point if the previous path is at the origin
    // and its angle is at zero degrees
    // this is done to make the math much easier
    for(int a = 0; a < ptsx.size(); a++){
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[a] - ref_x;
        double shift_y = ptsy[a] - ref_y;

        ptsx[a] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[a] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // set the (x, y) points to the spline
    spline.set_points(ptsx, ptsy);

    // define the actual (x, y) points we will use for the planner
    // these are the actual points that the path planner is going to use
    // vector<double> next_x_vals; // already defined
    // vector<double> next_y_vals; // already defined

    // start with all the previous path points from the last time
    // helps with the transition
    // instead of recreating the path from scratch every single time, we just add points onto it and
    // work with what you still had left from last time
    for(int a = 0; a < previous_path_x.size(); a++){
        // load up the future path with whatever was left over from the previous path
        // done for continuity
        next_x_vals.push_back(previous_path_x[a]);
        next_y_vals.push_back(previous_path_y[a]);
        // cout << "previous_path_x: " << previous_path_x[a] << " previous_path_y " << previous_path_y[a] << endl;
    }

    /*
     * from the car's point of view or car coordinate system where the car is the origin
     *
     * forward for the car will be going at zero degrees
     *
     * the spline will give us the points in the path that are spaced alone the spline and spaced in
     * such a way that the car will move at the desired speed from the car to a desired point 30m away
     *
     * we do this by looking at some horizon value, 30m away in front of the car in this case
     * we figure out where that point lies on the spline by plugging it into the spline to get the
     * output y value
     * we need to calculate the distance from the car to the outputted value
     * we now want to split up distance d into n pieces
     * n pieces * .02 sec / piece * desired velocity (m/s) = distance calculated
     * we use the above formula to calculate n
     *
     *
     */
    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = spacing;
    double target_y = spline(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    double x_add_on = 0; // has to do with the car local transformation; we start at the origin

    // fill up the rest of the path planner after filling it with previous points,
    // here we will always output 50 points
    // previous_path_x will not be 50 points
    // the simulator will probably go through a few of these points and report them the next time around
    // previous_path_x is not the full path, it is simply parts of it that the car did not move to yet
    for(int a = 1; a <= 50 - previous_path_x.size(); a++){
        // in path planner, when it is going through each point, we can change the ref_vel here
        // doing the speed increment here to be a little efficient
        // cout << ptsx[a] << " " << ptsy[a] << endl;
        // done here because we use the ref_vel value to plan out the future path
        // can take deceleration into account here
        if(ref_vel > speed){
            ref_vel -= speed_adjust_rate; // -= 5m/s if the vehicle is too close, decrease by .224 each time until its 30m away
            if(ref_vel < speed) ref_vel = speed; // instead of slowing completely to a stop, I'm gonna slow down just enough that I match the car in front of me
        } else if(ref_vel < speed_limit && speed == speed_limit){ // speed up if there is nothing in the way
            // if going too slow, incrementally speed up
            ref_vel += speed_adjust_rate; // += 5m/s
        }
        if(ref_vel >= speed_limit) ref_vel = speed_limit - .5;// modified to keep the car under the speed limit

        double n = (target_dist / (time_interval * ref_vel / mph_2_mps));// 2.24 m/s instead of 50mph
        double x_point = x_add_on + (target_x / n); // start from the origin and work up to the target_x value
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // transform back to normal coordinate points
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

vector<double> trajectory::getNext_x_vals(){
    return next_x_vals;
}

vector<double> trajectory::getNext_y_vals(){
    return next_y_vals;
}