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
#include "spline.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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

  int lane = 1;
  double ref_vel = 0;
  double cost[3] = {0, 0.5, 0};

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
            // auto sensor_fusion = j[1]["sensor_fusion"];
            int new_lane = lane; // set next lane to current
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
            int prev_size = previous_path_x.size();

            if (prev_size > 0)
            {
              car_s = end_path_s;
            }
            double rear_speed[3] = {0, 0, 0}; // speed behind
            double front_speed[3] = {50, 50, 50}; //speed ahead
            double min_dist_front[3] = {200, 200, 200}; // min dist to front
            double min_dist_rear[3] = {200, 200, 200}; // min dist to rear
            double min_expected_rear[3] = {200, 200, 200};
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {
              double check_x = sensor_fusion[i][1];
              double check_y = sensor_fusion[i][2];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_s = sensor_fusion[i][5];
              check_s += (double)prev_size*0.02*check_speed;
              double check_d = sensor_fusion[i][6];
              
              // rear vehicle
              if (car_s > check_s) 
              {
                if ((car_s-check_s < min_dist_rear[(int)check_d/4]))
                {
                  min_dist_rear[(int)check_d/4] = car_s-check_s;
                  rear_speed[(int)check_d/4] = check_speed;
                  min_expected_rear[(int)check_d/4] = min_dist_rear[(int)check_d/4] - (check_speed - car_speed)*2/2.24;
                }
              }
              //front vehicle
              else 
              {
                if ((check_s - car_s < min_dist_front[(int)check_d/4]))
                {
                  min_dist_front[(int)check_d/4] = check_s - car_s;
                  front_speed[(int)check_d/4] = check_speed;
                }
              }
              if (car_s-check_s < 50)
              {
                if (( car_s - check_s + (car_speed - check_speed)*2/2.24) < min_expected_rear[(int)check_d/4])
                {
                  min_expected_rear[(int)check_d/4] =  (car_s - check_s + (car_speed - check_speed)*2/2.24);
                }
              }
            }
            bool too_close = false;
            if (min_dist_front[lane] < 30)
            {
              too_close = true;
            }
            if (min_dist_front[lane] < 60) 
            {
              if (lane == 2 || lane == 0)
              {
                if (min_dist_front[1] > 30 && min_expected_rear[1] > 30)
                {
                  if (min_dist_front[lane] < 30)
                  {
                    if (front_speed[1] > front_speed[lane])
                    {
                      new_lane = 1;
                    }
                  }
                  else
                  {
                    new_lane = 1;
                  }
                }
              }
              else
              {
                if (min_dist_front[lane] < 30)
                {
                  if (((min_dist_front[2] > 60) ||((min_dist_front[2] > 45) && (front_speed[2] > front_speed[lane]))) && min_expected_rear[2] > 30)
                  {
                    if (front_speed[2] > front_speed[0])
                    {
                      new_lane = 2;
                    }
                    else
                    {
                      if (min_dist_front[2] > min_dist_front[0] + 60)
                      {
                        new_lane = 2;
                      }
                    }
                  }
                  if (((min_dist_front[0] > 60) || ((min_dist_front[0] > 45) && (front_speed[0] > front_speed[lane]))) && min_expected_rear[0] > 30)
                  {
                    if (front_speed[0] > front_speed[2])
                    {
                      new_lane = 0;
                    }
                    else
                    {
                      if (min_dist_front[0] > min_dist_front[2] + 60)
                      {
                        new_lane = 0;
                      }
                    }
                  }
                  if (min_dist_front[2] > 180 && min_expected_rear[2] > 30)
                  {
                    new_lane = 2;
                  }
                  if (min_dist_front[0] > 180 && min_expected_rear[0] > 30)
                  {
                    new_lane = 0;
                  }
                }
              }
            }
            if (min_dist_front[1] > 120 && min_expected_rear[1] > 30)
            {
              new_lane = 1;
            }

            if (lane != new_lane)
            {
              cout<<"shifting from lane "<<lane<<" to lane "<<new_lane<<endl;
            }
            lane = new_lane;
            if (too_close)
            {
              if (ref_vel > front_speed[lane])
              {
                ref_vel -= 0.224;
              }
              else
              {
                if (ref_vel < 49.5)
                {
                  ref_vel += 0.112;
                }
              }
            }
            else 
            {
              if(ref_vel < 49.5)
              {
                ref_vel += 0.224;
              }
            }
            vector<double> ptsx;
            vector<double> ptsy;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            if(prev_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);  
            }
            else
            {
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }
            vector<double> next_wayp[10];
            int step = 1;
            for (int i = 0; i < 2; ++i)
            {
              next_wayp[i] = getXY(car_s+30*step*(i+1), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            }
            for (int i = 0; i < 2; ++i)
            {
              ptsx.push_back(next_wayp[i][0]);
              ptsy.push_back(next_wayp[i][1]);
            }
            for (int i = 0; i < ptsx.size(); ++i)
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x*cos(ref_yaw) + shift_y*sin(ref_yaw);
              ptsy[i] = shift_x*sin(-1*ref_yaw) + shift_y*cos(ref_yaw);
            }
            tk::spline s;
            s.set_points(ptsx, ptsy);
            json msgJson;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for (int i = 0; i < previous_path_x.size(); ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            double target_x = 30;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            double x_add_on = 0;
            for (int i = 1; i <= 50 - previous_path_x.size(); ++i)
            {
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
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
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
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