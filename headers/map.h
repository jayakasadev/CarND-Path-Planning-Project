//
// Created by jay on 11/27/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

class map_data {
public:
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

    static map_data& getInstance(){
        static map_data map;
        return map;
    }

    void operator=(map_data const&) = delete;

private:
    // Waypoint map to read from
    const std::string map_file_ = "../data/highway_map.csv";
    map_data(){
        std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

        std::string line;
        while (getline(in_map_, line)) {
            std::istringstream iss(line);
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
    }
};


#endif //PATH_PLANNING_MAP_H
