#include <uWS/uWS.h>
#include <future>

#include "Eigen-3.3/Eigen/Dense"
#include "utilities/json.hpp"
#include "map/map.h"
#include "vehicle/vehicle.h"
#include "scores/scores.h"
#include "sensor_fusion/sensor_fusion.h"
#include "trajectory_generation/trajectory_generator.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

    map_data mapData;

    driver car;

    scores values(1);

    sensorfusion sensorFusion(car, values, mapData);

    behavior_planner behaviorPlanner(car, values);

    trajectory_generator trajectory(mapData);

    vector<trajectory_option> options;

    h.onMessage([&mapData, &car, &values, &sensorFusion, &trajectory, &behaviorPlanner, &options](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        // auto sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
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
                    auto sensor_fusion_data = j[1]["sensor_fusion"];

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    short size = previous_path_x.size();

                    // future<void> sf(async(launch::deferred, [&sensorFusion, &sensor_fusion_data]{sensorFusion.predict(sensor_fusion_data);})); // launch instantly
                    cout << "current\tcar_x: " <<  car_x << "\tcar_y: " <<  car_y << "\tcar_s: " << car_s << "\tcar_d: "
                         << car_d << "\tcar_yaw: " << car_yaw << "\tcar_speed: " << car_speed << endl;
                    car.update(car_x, car_y, car_s, car_d, deg2rad(car_yaw), car_speed * mph_to_mps);
                    car.print();
                    // reset the scores
                    values.reset(car.getLane());
                    sensorFusion.predict(sensor_fusion_data, previous_path_x, previous_path_y);

                    cout << "prev_size: " << size << endl;

                    if(size > 0){
                        // cars moving
                        // TODO get the last two points in the path and use them to calculate yaw, speed, s, d
                        // car.update(car_x, car_y, car_s, car_d, car_yaw, car_speed);
                        // look at the last couple points in the previous path that the car was following and then
                        // calculate what angle the car was heading in using those last couple of points
                        // then push these points onto a list of previous points

                        // use the previous path's endpoint as a starting reference
                        // Redefine reference state as previous path end point

                        double ref_x = previous_path_x[size - 1];
                        double ref_y = previous_path_y[size - 1];

                        // make sure the path is tangent to the the angle of the car by using the last two points in
                        // the previous path
                        double ref_x_prev = previous_path_x[size - 2];
                        double ref_y_prev = previous_path_y[size - 2];

                        // double ref_x_prev_prev = previous_path_x[size - 3];
                        // double ref_y_prev_prev = previous_path_y[size - 3];

                        // cout << "ref: [ " << ref_x << ", " << ref_y << " ]\t";
                        // cout << "prev: [ " << ref_x_prev << ", " << ref_y_prev << " ]" << endl;
                        // cout << "\t[ s = " << end_path_s << ", d = " << end_path_d << " ]" << endl;

                        double ref_yaw = calculateYaw(ref_y, ref_y_prev, ref_x, ref_x_prev);
                        // cout << "calculated yaw in radians: " << ref_yaw << endl;
                        cout << "\tcalculated yaw in degrees: " << rad2deg(ref_yaw) << endl;

                        // double ref_yaw2 = calculateYaw(ref_y_prev, ref_y_prev_prev, ref_x_prev, ref_x_prev_prev);
                        // cout << "calculated yaw in radians: " << ref_yaw << endl;
                        // cout << "\tcalculated yaw2 in degrees: " << rad2deg(ref_yaw2) << endl;

                        vector<double> sAndD = mapData.getFrenet(ref_x, ref_y, rad2deg(ref_yaw));
                        cout << "\t[ s = " << sAndD[0] << ", d = " << sAndD[1] << " ]" << endl;
                        // cout << "\t[ s = " << end_path_s << ", d = " << end_path_d << " ]" << endl;

                        double velocity = sqrt(pow(((ref_x - ref_x_prev) / refresh_rate), 2) + pow(((ref_y - ref_y_prev) / refresh_rate), 2));
                        cout << "\tcalculated velocity: " << velocity << endl;
                        // double velocity2 = sqrt(pow(((ref_x_prev - ref_x_prev_prev) / refresh_rate), 2) + pow(((ref_y_prev - ref_y_prev_prev) / refresh_rate), 2));
                        // cout << "\tcalculated velocity2: " << velocity2 << endl;
                        // double acceleration = ((velocity - velocity2) / refresh_rate);
                        // cout << "\tacceleration: " << acceleration << endl;

                        // cout << "\tvelocity based on s vals: " << (end_path_s - sAndD[0]) / refresh_rate << endl;

                        car.update(ref_x, ref_y, sAndD[0], sAndD[1], ref_yaw, velocity);
                        // car.update(ref_x, ref_y, end_path_s, end_path_d, ref_yaw, velocity);
                        // car.setAccelerationS(acceleration);
                        car.print();
                        // cout << "\tacceleration_f: " << options[0].acceleration << endl;
                        // cout << "\tvelocity_f: " << options[0].velocity << endl;


                        // cout << "old points" << endl;
                        for(short a = 0; a < size; a++){
                            next_x_vals.push_back(previous_path_x[a]);
                            next_y_vals.push_back(previous_path_y[a]);
                            cout << "prev [ " << next_x_vals[a] << ", " << next_y_vals[a] << " ]" << endl;
                        }
                        return;
                    }

                    // cout << "setup car" << endl;
                    // car.update(car_x, car_y, car_s, car_d, car_yaw, car_speed);
                    // cout << "finised reset" << endl;

                    // thread to run sensor_fusion
                    // sensorFusion.predict(sensor_fusion_data);
                    // thread sf(&sensorfusion::predict, sensorFusion, sensor_fusion_data);
                    // consider a copy on write buffer for scores

                    // cout << "finished prediction" << endl;

                    // thread to run behavior_planner
                    // future<vector<trajectory_option>> bp(async(launch::deferred, [&behaviorPlanner]{return behaviorPlanner.plan();}));
                    // sf.get();
                    // behaviorPlanner.plan(); // run on main thread

                    options = behaviorPlanner.plan();

                    trajectory.calculatePoints(options[0], options[1], size);

                    vector<double> x_vals = trajectory.getXVals();
                    vector<double> y_vals = trajectory.getYVals();

                    // cout << "new points" << endl;
                    for(short a = 0; a < x_vals.size(); a++){
                        next_x_vals.push_back(x_vals[a]);
                        next_y_vals.push_back(y_vals[a]);

                        cout << "new [ " << x_vals[a] << ", " << y_vals[a] << " ]" << endl;
                    }

                    json msgJson;

                    // cout << "next_x_vals size : " << next_x_vals.size() << " next_y_vals size : " << next_y_vals.size() << endl;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    // cout << "set x and y vals" << endl;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    // cout << "end of block" << endl;
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
        // cout << "done" << endl;
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
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
