#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h" // added to include the spline library

using namespace std;

// for convenience
using json = nlohmann::json;

// fixed speed limit
const double speed_limit = 50.0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Checks if the SocketIO event has JSON data. If there is data the JSON object in string format will be returned, else
 * the empty string "" will be returned.
 *
 * @param s
 * @return
 */
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
    return "";
    } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

/**
 * Calculate distance between two points
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * Gives you the id of the closest waypoint to the vehicle
 *
 * @param x
 * @param y
 * @param maps_x
 * @param maps_y
 * @return
 */
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

/**
 * Gives the id of the next waypoint
 *
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return
 */
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

/**
 * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 *
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return
 */
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0){
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef){
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++){
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )){
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

      // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
  	    istringstream iss(line);
  	    double x;
  	    double y;
  	    float s;
  	    float d_x;
  	    float d_y;
  	    iss >> x;
  	    iss >> y;
  	    iss >> s;
  	    iss >> d_x;
  	    iss >> d_y;
  	    map_waypoints_x.push_back(x);
  	    map_waypoints_y.push_back(y);
  	    map_waypoints_s.push_back(s);
  	    map_waypoints_dx.push_back(d_x);
  	    map_waypoints_dy.push_back(d_y);
    }

    // start in lane 1
    int lane = 1; // lane 0 is far left, lane 2 is far right

    // reference velocity to target
    double ref_vel = 0.0; // start from 0.0 mph and incrementally speed up instead of instantly jumping to 50mph


    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane,
                        &ref_vel, &speed_limit](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);
        
                string event = j[0].get<string>();
                // this is where the action happens
                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    // given by simulator
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];

                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    // as long as these vectors are empty, the car will not go anywhere
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    /*
                     * move the car forward in a straight line at a constant 50 MPH velocity. Use the car's (x, y)
                     * localization information and its heading direction to create a simple, straight path that is
                     * drawn directly in front of the car.
                     * car went from 0 MPH to 56 MPH in a single 20 ms frame, causing a spike in acceleration.
                     *
                     * Acceleration is calculated by comparing the rate of change of average speed over .2 second
                     * intervals. In this case total acceleration at one point was as high as 75 m/s^2. Jerk was also
                     * very high. The jerk is calculated as the average acceleration over 1 second intervals. In order
                     * for the passenger to have an enjoyable ride both the jerk and the total acceleration should not
                     * exceed 10 m/s^2.
                     *
                     * Part of the total acceleration is the normal component, AccN which measures the centripetal
                     * acceleration from turning. The tighter and faster a turn is made, the higher the AccN value will
                     * be.
                     */
                    /*
                    // driving using x, y coordinates in a straight line
                    double dist_inc = 0.5;
                    for(int i = 0; i < 50; i++) {
                        next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                        next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
                    }
                    */
                    /*
                    // going to try to go straight but stay in my lane using the s and d coordinate
                    double dist_inc = 0.5; // cover less distance on each step to lower the top speed
                    for(int i = 0; i < 50; i++) {
                        double next_s = car_s + (i + 1) * dist_inc; // start at i+1 because we want the next position,
                        // and i is exactly where the car is, so there will be no transition, it will sit still
                        double next_d = 6;
                        // we start at 1 and a half lane away from the road boundaries
                        // each lane is 4 meters wide, so 1.5 lanes * 4 meters = 6 meters away from the road boundary to
                        // stay in the center of the lane

                        // convert to x,y coordinates
                        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                    }
                     */
                    /*
                     * path needs to be smoothed out to handle the excessive jerk issue
                     * will use the spline library to do this instead of polynomial fit
                     * this is done because all the code that i would need to implement for polynomial fit is already
                     * implemented in spline.h
                     */
                    /*
                    for(int a = 0; a < next_x_vals.size(); a++){
                        cout << "next_x_vals: " << next_x_vals[a] << " next_y_vals: " << next_y_vals[a] << endl;
                    }
                     */

                    int prev_size = previous_path_x.size();

                    // we want the car to drive in a single lane and smoothly at a constant velocity

                    // working on sensor fusion here to avoid obstacles
                    if(prev_size > 0){
                        // if i have any previous path points. going to change my current s so that it is actually representative of the previous path last point's s
                        // done in frenet because it simplifies the logic
                        car_s = end_path_s;
                    }

                    bool too_close = false;

                    // find ref_v to use
                    for(int a = 0; a < sensor_fusion.size(); a++){
                        // car is in my lane
                        // looking at the ath car on the road
                        float d = sensor_fusion[a][6]; // this is the d value for the ath vehicle
                        // cout << "a: " << a << " d: " << d << endl;
                        if(d < (4 + 4 * lane) && d < (4 * lane)){ // checking if the car is in the range of my lane
                            double vx = sensor_fusion[a][3]; // lateral velocity
                            double vy = sensor_fusion[a][4]; // longitudinal velocity
                            double check_speed = sqrt(pow(vx, 2) + pow(vy, 2)); // velocity
                            double check_car_s = sensor_fusion[a][5]; // s value of the ath car
                            // cout << "\tcheck_car_s: " << check_car_s << endl;

                            // if using previous points can project s value out
                            check_car_s += ((double) prev_size * .02 * check_speed);// predicting where the car will be (.02 * prev points) seconds in the future
                            // cout << "\tfuture check_car_s: " << check_car_s << endl;
                            // cout << "\tcar_s: " << car_s << endl;

                            // check s values greater than mine and s gap
                            if((check_car_s > car_s) && ((check_car_s - car_s) < 30)) { // check if the car will be within 30m of my car in the future
                                // do some logic here, lower reference velocity so we don't crash into the car in front of us, could also flag to try to change lanes
                                // ref_vel = 29.5; // mph // set speed change
                                too_close = true; // flag used to make incremental changes to the speed
                            }
                        }
                    }

                    // cout << "ref_vel: " << ref_vel << endl;

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
                    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 30m
                    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 60m
                    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // car location in 90m

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

                        // todo experiment with speed placement
                        // doing the speed increment here to be a little efficient
                        // cout << ptsx[a] << " " << ptsy[a] << endl;
                        if(too_close && ref_vel > 0){
                            ref_vel -= .224; // -= 5m/s if the vehicle is too close, decrease by .224 each time until its 30m away
                        } else if(ref_vel < speed_limit){
                            // if going too slow, incrementally speed up
                            ref_vel += .224; // += 5m/s
                        }
                    }
                    if(ref_vel > speed_limit) ref_vel = speed_limit - .05; // safety check to make sure i don't go over the speed limit
                    if(ref_vel < 0) ref_vel = 0.0; // safety check to keep the car from going backwards or having negative velocity

                    // create a spline
                    tk::spline spline;

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
                    double target_x = 30.0;
                    double target_y = spline(target_x);
                    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

                    double x_add_on = 0; // has to do with the car local transformation; we start at the origin

                    // fill up the rest of the path planner after filling it with previous points,
                    // here we will always output 50 points
                    // previous_path_x will not be 50 points
                    // the simulator will probably go through a few of these points and report them the next time around
                    // previous_path_x is not the full path, it is simply parts of it that the car did not move to yet
                    for(int a = 1; a <= 50 - previous_path_x.size(); a++){
                        double n = (target_dist / (.02 * ref_vel / 2.24));// 2.24 m/s instead of 50mph
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

                    /*
                    for(int a = 0; a < next_x_vals.size(); a++){
                        cout << "next_x_vals: " << next_x_vals[a] << " next_y_vals " << next_y_vals[a] << endl;
                    }
                     */


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    /**
     * We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
     */
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;

    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}