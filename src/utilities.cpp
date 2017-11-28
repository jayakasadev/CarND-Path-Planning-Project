#include "../headers/utilities.h"

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
int ClosestWaypoint(double x, double y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < map.map_waypoints_x.size(); i++)
    {
        double map_x = map.map_waypoints_x[i];
        double map_y = map.map_waypoints_y[i];
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
int NextWaypoint(double x, double y, double theta) {

    int closestWaypoint = ClosestWaypoint(x,y);

    double map_x = map.map_waypoints_x[closestWaypoint];
    double map_y = map.map_waypoints_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );
    double angle = fabs(theta - heading);

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
vector<double> getFrenet(double x, double y, double theta) {
    int next_wp = NextWaypoint(x,y, theta);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0){
        prev_wp  = map.map_waypoints_x.size()-1;
    }

    double n_x = map.map_waypoints_x[next_wp]-map.map_waypoints_x[prev_wp];
    double n_y = map.map_waypoints_y[next_wp]-map.map_waypoints_y[prev_wp];
    double x_x = x - map.map_waypoints_x[prev_wp];
    double x_y = y - map.map_waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-map.map_waypoints_x[prev_wp];
    double center_y = 2000-map.map_waypoints_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef){
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++){
        frenet_s += distance(map.map_waypoints_x[i],map.map_waypoints_y[i],map.map_waypoints_x[i+1],map.map_waypoints_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d) {
    int prev_wp = -1;

    while(s > map.map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map.map_waypoints_s.size()-1) )){
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%map.map_waypoints_x.size();

    double heading = atan2((map.map_waypoints_y[wp2]-map.map_waypoints_y[prev_wp]),(map.map_waypoints_x[wp2]-map.map_waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-map.map_waypoints_s[prev_wp]);

    double seg_x = map.map_waypoints_x[prev_wp]+seg_s*cos(heading);
    double seg_y = map.map_waypoints_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

inline double getYaw(double ref_y, double ref_y_prev, double ref_x, double ref_x_prev){
    return atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
}

inline int min_element(float arr[], int size){
    int index = 0;
    // cout << "elements: " << size << endl;
    for(int a = 1; a < size; a++){
        if(arr[index] > arr[a]){
            index = a;
        }
    }
    return index;
}