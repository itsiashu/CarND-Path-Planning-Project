#ifndef PATH_PLANNING_H
#define PATH_PLAN_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// test for speed reduction
bool verify_speed_reduction(const vector<vector<double>> &sensor_fusion, int cur_lane, double end_path_s, double car_s) {
  bool speed_reduction = false;
  for(int i=0; i< sensor_fusion.size(); i++) {
    // if cars present in our lane
    if((sensor_fusion[i][6] > (2+4*cur_lane)-2) && (sensor_fusion[i][6] < (2+4*cur_lane)+2)) {
      // reduce speed, if car infront is at less then 30 m
      if(sensor_fusion[i][5] < (30 + end_path_s) && (sensor_fusion[i][5] > car_s)) {
        speed_reduction = true;
      }
    }
  }
  return speed_reduction;
}

// if left lane ok for cars
bool check_left_lane(const vector<vector<double>> &sensor_fusion, int cur_lane, double car_s) {
  bool left_ok = false;
  int index = 0;
  int sf_size = sensor_fusion.size();
  
  if (cur_lane > 0) {
    cur_lane--;
    // check for lane change
    for (index = 0; index < sf_size; index++) {
      if ((sensor_fusion[index][6] > (2 + 4 * cur_lane) - 2) && (sensor_fusion[index][6] < (2 + 4 * cur_lane) + 2)) {
        // if front car is 25m ahead and rear car is 5m behind
        if (sensor_fusion[index][5] < (25 + car_s) && (sensor_fusion[index][5] > (car_s - 5))) {
          break;
        }
      }
    }
  }
  if(index+1 == sf_size)
        left_ok = true;
  return left_ok;
}


// if right lane ok for cars
bool check_right_lane(const vector<vector<double>> &sensor_fusion, int cur_lane, double car_s, bool left_ok) {
  bool right_ok = false;
  int index = 0;
  int sf_size = sensor_fusion.size();
  
  if (cur_lane < 2 && !left_ok) {
    cur_lane++;
    // check for lane change
    for (index = 0; index < sf_size; index++) {
      if ((sensor_fusion[index][6] > (2 + 4 * cur_lane) - 2) && (sensor_fusion[index][6] < (2 + 4 * cur_lane) + 2)) {
        // if front car is 25m ahead and rear car is 5m behind
        if (sensor_fusion[index][5] < (25 + car_s) && (sensor_fusion[index][5] > (car_s - 5))) {
          break;
        }
      }
    }
  }
  if(index+1 == sf_size)
        right_ok = true;
  return right_ok;
}

// store next-waypoints for trajectory generation
void populate_waypoints(vector<double> &points_x, vector<double> &points_y, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, double end_path_s, double ref_yaw, double ref_x, double ref_y, int cur_lane) {
  vector<double> way_point0 = getXY(end_path_s + 30, (2 + 4 * cur_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> way_point1 = getXY(end_path_s + 60, (2 + 4 * cur_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> way_point2 = getXY(end_path_s + 90, (2 + 4 * cur_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  points_x.push_back(way_point0[0]);
  points_x.push_back(way_point1[0]);
  points_x.push_back(way_point2[0]);

  points_y.push_back(way_point0[1]);
  points_y.push_back(way_point1[1]);
  points_y.push_back(way_point2[1]);

  // for spline fitting change point to frenet
  for(int i=0; i< points_x.size(); i++) {
    // shift reference angle to zero
    double shift_x = points_x[i] - ref_x;
    double shift_y = points_y[i] - ref_y;
    // and rotate
    points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }
}

void build_path(double ref_vel, const vector<double> &previous_path_x, const vector<double> &previous_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals, double ref_yaw, double ref_x, double ref_y, tk::spline sp) {
  for(int i=0; i< previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  // get in-between points 
  double target_x = 30.0;
  double target_y = sp(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
  double add_x = 0;

  // fill for 50 points
  for(int i = 1; i < 50-previous_path_x.size(); i++) {
    double N = (target_dist/ (0.02 * ref_vel / 2.24));
    double x_pt = add_x + (target_x)/N;
    double y_pt = sp(x_pt);
    add_x = x_pt;
    double x_ref = x_pt;
    double y_ref = y_pt;
    // from frenet to global
    x_pt = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_pt = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    // translate
    x_pt += ref_x;
    y_pt += ref_y;
    next_x_vals.push_back(x_pt);
    next_y_vals.push_back(y_pt);
  }
}
#endif  // PATH_PLANNING_H

