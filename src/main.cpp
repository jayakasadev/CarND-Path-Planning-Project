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

    trajectory_option option;

    behavior_planner behaviorPlanner(car, values, option);

    trajectory_generator trajectory(mapData, option);


    h.onMessage([&mapData, &car, &values, &sensorFusion, &trajectory, &behaviorPlanner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
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

                    // future<void> sf(async(launch::deferred, [&sensorFusion, &sensor_fusion_data]{sensorFusion.predict(sensor_fusion_data);})); // launch instantly
                    car.update(car_x, car_y, car_s, car_d, car_yaw, car_speed);
                    car.print();
                    // reset the scores
                    values.reset(car.getLane());
                    sensorFusion.predict(sensor_fusion_data, previous_path_x, previous_path_y);

                    int size = previous_path_x.size();

                    if(size > 0 || values.getBehavior(car.getLane()) == STOP){
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

                        double ref_yaw = calculateYaw(ref_y, ref_y_prev, ref_x, ref_x_prev);

                        vector<double> sAndD = mapData.getFrenet(ref_x, ref_y, rad2deg(ref_yaw));

                        double velocity = sqrt(pow(((ref_x - ref_x_prev)/ refresh_rate), 2) + pow(((ref_y - ref_y_prev)/ refresh_rate), 2));

                        cout << "car_x: " << car_x << "\tcar_y: " << car_y << "\tcar_s: " << car_s << "\tcar_d: "
                             << car_d << "\tcar_yaw: " << car_yaw << "\tcar_speed: " << car_speed << endl;
                        cout << "calculated velocity: " << velocity << endl;

                        car.update(ref_x, ref_y, sAndD[0], sAndD[1], ref_yaw, velocity);
                        car.print();
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
                    // future<vector<VectorXd>> bp(async([&behaviorPlanner]{return behaviorPlanner.bestOption();}));
                    // sf.get();
                    behaviorPlanner.bestOption(); // run on main thread

                    trajectory.calculatePoints();

                    json msgJson;

                    vector<double> next_x_vals = trajectory.getXVals();
                    vector<double> next_y_vals = trajectory.getYVals();

                    // next_x_vals.push_back(car_x);
                    // next_x_vals.push_back(car_y);

                    cout << "next_x_vals size : " << next_x_vals.size() << " next_y_vals size : " << next_y_vals.size() << endl;

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
