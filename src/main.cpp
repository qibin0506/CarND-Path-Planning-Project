#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// can turn to this lane
bool can_turn(int prev_size, double car_s, int index, vector<vector<double>> sensor_fusion) {
  double vx = sensor_fusion[index][3];
  double vy = sensor_fusion[index][4];
  double check_speed = sqrt(vx * vx + vy * vy);
  double check_s = sensor_fusion[index][5];

  check_s += prev_size * 0.02 * check_speed;
  if (abs(check_s - car_s) < 30.0) {
    return false;
  }
  
  return true;
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
  
  double lane = 1;
  double ref_velocity = 0.0;
  
  h.onMessage([&ref_velocity, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          bool too_close = false;
          bool turn_left = false;
          bool turn_right = false;
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          for (int i = 0; i < sensor_fusion.size(); i++) {
            double d = sensor_fusion[i][6];
            if (d > 2 + 4 * lane - 2 && d < 2 + 4 * lane + 2) {
              double vx = sensor_fusion[i][3];
  			  double vy = sensor_fusion[i][4];
  			  double check_speed = sqrt(vx * vx + vy * vy);
  			  double check_s = sensor_fusion[i][5];

  			  check_s += prev_size * 0.02 * check_speed;
  			  if (check_s > car_s && check_s - car_s < 30.0) {
    			too_close = true;
                break;
  			  }
            }
          }
          
          if (too_close) {
          	for (int j = 0; j < sensor_fusion.size(); j++) {
            	double dd = sensor_fusion[j][6];
                
                if (lane == 0) { // now on the left lane
                  // check lane 1 for turn right
                  if (dd > 2 + 4 * 1 - 2 && dd < 2 + 4 * 1 + 2) {
                    turn_right = can_turn(prev_size, car_s, j, sensor_fusion);
                  }
                } else if (lane == 1) { // now on the center lane
                  if (dd > 2 + 4 * 0 - 2 && dd < 2 + 4 * 0 + 2) { // check lane 0 for turn left
                    turn_left = can_turn(prev_size, car_s, j, sensor_fusion);
                  } else if (dd > 2 + 4 * 2 - 2 && dd < 2 + 4 * 2 + 2) { // check lane 2 for turn right
                    turn_right = can_turn(prev_size, car_s, j, sensor_fusion);
                  }
                } else if (lane == 2) { // now on the right lane
                  if (dd > 2 + 4 * 0 - 2 && dd < 2 + 4 * 0 + 2) { // check lane 0 for turn left
                    turn_left = can_turn(prev_size, car_s, j, sensor_fusion);
                  }
                }
            }
          }
 
          if (turn_right) {
            lane += 1;
          } else if (turn_left) {
            lane -= 1;
          } else if (too_close) {
            ref_velocity -= 0.224;
          } else if (ref_velocity < 49.5) {
            ref_velocity += 0.224;
          }
          
          vector<double> points_x;
          vector<double> points_y;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // start using previous car points
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            points_x.push_back(prev_car_x);
            points_x.push_back(car_x);
            
            points_y.push_back(prev_car_y);
            points_y.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double prev_car_x = previous_path_x[prev_size - 2];
            double prev_car_y = previous_path_y[prev_size - 2];
            
            ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
            
            points_x.push_back(prev_car_x);
            points_x.push_back(ref_x);
            
            points_y.push_back(prev_car_y);
            points_y.push_back(ref_y);
          }
          // end
          
          // start using next way points
          vector<double> next_waypoints_0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoints_1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoints_2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          points_x.push_back(next_waypoints_0[0]);
          points_x.push_back(next_waypoints_1[0]);
          points_x.push_back(next_waypoints_2[0]);
          
          points_y.push_back(next_waypoints_0[1]);
          points_y.push_back(next_waypoints_1[1]);
          points_y.push_back(next_waypoints_2[1]);
          // end
          
          // start using the current status as the base point
          for (int i = 0; i < points_x.size(); i++) {
            double shift_x = points_x[i] - ref_x;
            double shift_y = points_y[i] - ref_y;
            
            points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          // end
          
          
          // start add the previous points
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // end
          
          tk::spline s;
          s.set_points(points_x, points_y);
          
          // target_x is base on current status
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);
          
          double x_add_on = 0;
          for (int i = 1; i < 50 - previous_path_x.size(); i++) {
            // distance = N * time_per_update * velocity
            // velocity / 2.24 is converting to km/h
            double N = target_distance / (0.02 * ref_velocity / 2.24);
            double point_x = x_add_on + target_x / N;
            double point_y = s(point_x);
            
            x_add_on = point_x;
            
            // converting from local coordinator to global coordinator
            double x_ref = point_x;
            double y_ref = point_y;
            
            point_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            point_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            point_x += ref_x;
            point_y += ref_y;
            
            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
          }
          
//           double incr = 0.5;
//           for (int i = 0; i < 50; i++) {
//             double next_s = car_s + (i + 1) * incr;
//             double next_d = 1.5 * 4;
//             vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//             next_x_vals.push_back(xy[0]);
//             next_y_vals.push_back(xy[1]);
//           }
          
//           for (int i = 0; i < next_x_vals.size(); i++) {
//             std::cout << next_y_vals[i] << " ";
//           }
//           std::cout << std::endl;

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