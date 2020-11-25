#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "path_planning.h"
#include "json.hpp"
#include "spline.h"

using nlohmann::json;

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
  // in mph
  double ref_vel = 0.0;
  // start with middle-lane
  int cur_lane = 1; 
    
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&cur_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // use previous points to build next ones!
          vector<double> points_x;
          vector<double> points_y;

          int prev_size = previous_path_x.size();
          bool left_ok = false;
          bool right_ok = false;

          // to move car initially
          if(!ref_vel)
            end_path_s = car_s;
        
          bool speed_reduction = verify_speed_reduction(sensor_fusion, cur_lane, end_path_s, car_s);
          if (speed_reduction)
            ref_vel -= 0.224;
          // keep 49.5 as max
          else if (ref_vel < 49.5) 
            ref_vel += 0.224;

          int ref_lane = cur_lane;
          // If reduced speed, check cars in other lanes
          if (speed_reduction) {
            left_ok = check_left_lane(sensor_fusion, ref_lane, car_s);
            right_ok = check_right_lane(sensor_fusion, ref_lane, car_s, left_ok);
          }
          // if available, go left
          if (left_ok) {
            cur_lane = ref_lane - 1;
          } else if (right_ok) {
            cur_lane = ref_lane + 1;
          } else {
            cur_lane = ref_lane;
          }
          // define some reference states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // prev car state empty when initializing
          if(prev_size < 2) {
            // use current (x, y) and heading angle
            double car_x_prev = car_x - cos(car_yaw);
            double car_y_prev = car_y - sin(car_yaw);
            points_x.push_back(car_x_prev);
            points_y.push_back(car_y_prev);
            points_x.push_back(car_x);
            points_y.push_back(car_y);
          } 
          else {
            // more previous (x, y) points to refer 
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            // get a previous point to this too to get the tangent angle
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            // calculate what angle the car was moving in
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev );
            points_x.push_back(ref_x_prev);
            points_x.push_back(ref_x);
            points_y.push_back(ref_y_prev);
            points_y.push_back(ref_y);
          }
          // calculate next way-points
          populate_waypoints(points_x, points_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, end_path_s, ref_yaw, ref_x, ref_y, cur_lane);
          
          // next, create a spline
          tk::spline sp;
          sp.set_points(points_x, points_y);
          
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          build_path(ref_vel, previous_path_x, previous_path_y, next_x_vals, next_y_vals, ref_yaw, ref_x, ref_y, sp);
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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