#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "utilities/json.hpp"
#include "utilities/utilities.h"
#include "trajectory_generation/trajectory.h"
#include "sensor_fusion/sensor_fusion.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

int main() {
    uWS::Hub h;

    // start in lane 1
    // short lane = 1; // lane 0 is far left, lane 2 is far right

    // reference velocity to target

    // trajectory generator
    trajectory traj;
    sensor_fusion sensor;

    driver vehicle;
    scores scores; // there are 3 lanes at all times

    map_data mapData;
    bool first = true;

    h.onMessage([&vehicle, &mapData, &traj, &sensor, &scores, &first](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if(length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                // this is where the action happens
                if (j[0].get<string>() == "telemetry") {
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
                    /*
                    if(prev_size > 0){
                        // if i have any previous path points. going to change my current s so that it is actually
                        // representative of the previous path last point's s
                        // done in frenet because it simplifies the logic
                        vehicle.updateVehicle(car_speed, car_x, car_y, end_path_s, car_d, mapData);
                    }
                    else{
                        vehicle.updateVehicle(car_speed, car_x, car_y, car_s, car_d, mapData);
                    }
                    */
                    if(first){
                        vehicle.updateVehicle(car_speed, car_x, car_y, car_s, car_d, mapData);

                        // bool too_close = false;
                        // double speed = speed_limit;

                        sensor.calculateCost(sensor_fusion, vehicle, scores, mapData);

                        // gotta change lanes
                        // lane = sensor.getLane();

                        // vector<double> xy = mapData.getXY(vehicle.getS(), vehicle.getD());
                        // cout << "DRIVER:  || s: " << vehicle.getS() << " || d: " << vehicle.getD() << endl;
                        // cout << "x: " << xy[0] << " || y: " << xy[1] << "\n" << endl;

                        traj.generate(vehicle, scores, mapData);
                        first = false;
                    }

                    msgJson["next_x"] = traj.getNext_x_vals();
                    msgJson["next_y"] = traj.getNext_y_vals();

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

    h.onDisconnection([&h, &vehicle](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

double sensor_fusion(){

}