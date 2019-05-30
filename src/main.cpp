#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"  // aditional library for smooth trajectory generation
#include <cmath> // for exp calculation

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  // declaration & initialization
  double ref_vel = 0.01;  //initialize ref_vel, a reference velocity to targe, 0.01 to avoid zero-devide
  int lane = 1; //initialize lane, #0 = left lane, #1 = mid lane, #2 = right lane
  bool lc = false;  // when keeping lane:false, when lane changing:true
  int status = 1; // initialize status
  double waiting_counter = 0; //[sec]


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane,&status,&lc,&waiting_counter]
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

          int prev_size = previous_path_x.size();
          
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          

          // Sensor Fusion
          if(prev_size>0){
            car_s = end_path_s;
          }          
          
          //initialization for every 20 msec
          bool too_close = false; // car in front is too close
          bool no_lcl = false; // feasibility of lane change left, true = no lc chance
          bool no_lcr = false; // feasibility of lane change right, true = no lc chance
          
          ////
          //scan all the detected car
          ////
          float left_edge  = 2 + lane * 4 - 2;  // d-value of left side of my lane
          float right_edge = 2 + lane * 4 + 2;  // d-value of right side of my lane
          double diff_speed = 0; // max speed difference[mph] between my car and car ahead of 30m or less, assuming my car is faster than object car.
          double dist = 100; // min distance[m] between my car and car ahead of 30m or less.
          double slow_l = 100;  // slowest speed [mph] in left lane
          double slow_r = 100;  // slowest speed [mph] in right lane
          double dist_l = 100;  // clear distance of left lane
          double dist_r = 100;  // clear distance of right lane
          
          for(int i=0;i<sensor_fusion.size();i++){
            double vx = sensor_fusion[i][3];  // meter/sec
            double vy = sensor_fusion[i][4];  // meter/sec
            double check_speed = sqrt(vx*vx+vy*vy); // meter/sec 
            double check_car_s = sensor_fusion[i][5];
            float d = sensor_fusion[i][6];
            check_car_s += (double)prev_size * 0.02 * check_speed; // job cycle=0.02sec

            // Collision danger detection in a same lane
            if( (left_edge < d) && ( d < right_edge) ){  // ith car is in my lane ?
              if( (check_car_s > car_s) && ((check_car_s - car_s) < 30.0 ) ){
                too_close = true; // when car in front is within 30m, true while dangerous situation
                if((check_car_s - car_s) < dist){dist = check_car_s - car_s;} //min distance 
                if((car_speed - check_speed*2.24) > diff_speed){diff_speed = car_speed - check_speed*2.24;} //max speed difference 
              }
            }
            // check the feasibility of left lane change
            if( (lane > 0) && ((left_edge -4) < d) && (d < (right_edge -4) ) ){  // ith car is in left adjacentlane
              if( (check_car_s > car_s) && ((check_car_s - car_s) < dist_l) ){dist_l = check_car_s - car_s;} // check clear distance of leftlane
              if( (check_car_s > car_s) && (check_speed*2.24 < slow_l) ){slow_l = check_speed*2.24 - car_speed;} // check the slowest moving car in the left lane
              if(( (check_car_s - car_s) < 20.0 ) && (-10.0 < (check_car_s - car_s))){no_lcl = true;} // if fast moving car is within 30m range,20m in front or 10m in the back
            }
            // check the feasibility of right lane change
            if( (lane < 2) && (left_edge +4 <d) && (d < right_edge +4) ){  // ith car is in right adjacentlane
              if( (check_car_s > car_s) && ((check_car_s - car_s) < dist_r) ){dist_r = check_car_s - car_s;} // check clear distance of right lane
              if( (check_car_s > car_s) && (check_speed*2.24 < slow_r) ){slow_r = check_speed*2.24 - car_speed;} // check the slowest moving car in the right lane
              if(( (check_car_s - car_s) < 20.0 ) && (-10.0 < (check_car_s - car_s))){no_lcr = true;} // if fast moving car is within 30m range,20m in front or 10m in the back
            }
          }
          ////
          // end of scan all detected cars
          ////
          
          ////
          // Finite State Machine
          ////
          double d_fin;  // final d-value after lane change
          double acc = 0.224;  //+0.224 mph/20ms = +4.44m/s^2
          double brake = 0.224+0.224*(1-exp(- (diff_speed/20)*(30/dist) ) );  //assumption: relative speed < 20mph, dist > 30m 

          switch (status){
            case 1: //driving in mid lane
              if(too_close || waiting_counter > 10.0){ //when car in front is too close or reducing speed for more than 10 seconds 
                if((!no_lcl)&&(!no_lcr)){ // both right and left lane is OK to change
                  if(dist_l*slow_l > dist_r*slow_r){status = 5;waiting_counter = 0;} //left lane is more clear and moving faster
                  else{status = 2;waiting_counter = 0;} // right lane is more clear and moving faster
                }
                else if (!no_lcl) { //only left lane is OK to move
                  status = 5;
                  waiting_counter = 0;
                } // status change to lane change left
                else if (!no_lcr) {
                  status = 2;
                  waiting_counter = 0;
                } // status change to lane change right
                else {
                  ref_vel -= brake;
                  waiting_counter += 0.02;  //20ms
                } //slow down to avoid collision
              }
              else if (ref_vel < 49.5) {
                ref_vel += acc;
              }
              break;
            case 2:
              if (!lc) {
                d_fin = 10.0;
                lc = true;
                lane = 2;
              }
              if ( abs(car_d - d_fin) < 0.2 ) {
                lc = false;
                status = 3;
              }
              break;
            case 3:
              if(too_close || waiting_counter > 10.0){
                if(!no_lcl){
                  status = 4;
                  waiting_counter = 0;
                } // status change to lane change left
                else{
                  ref_vel -= brake;
                  waiting_counter += 0.02;  //20ms
                } //slow down to avoid collision
              }
              else if(ref_vel < 49.5){
                ref_vel += acc;
              }
              break;
            case 4:
              if(!lc){
                d_fin = 6.0;
                lc = true;
                lane = 1;
              }
              if( abs(car_d - d_fin) < 0.2 ){
                lc = false;
                status = 1;
              }
              break;
            case 5:
              if(!lc){
                d_fin = 2.0;
                lc = true;
                lane = 0;
              }
              if( abs(car_d - d_fin) < 0.2 ){
                lc = false;
                status = 6;
              }
              break;
            case 6:
              if(too_close || waiting_counter > 10.0){
                if(!no_lcr){
                  status = 7;
                  waiting_counter = 0;
                } // status change to lane change right
                else{
                  ref_vel -= brake;
                  waiting_counter += 0.02;  //20ms
                } //slow down to avoid collision
              }
              else if(ref_vel < 49.5){
                ref_vel += acc;
              }
              break;
            case 7:
              if(!lc){
                d_fin = 6.0;
                lc = true;
                lane = 1;
              }
              if( abs(car_d - d_fin) < 0.2 ){
                lc = false;
                status = 1;
              }
              break;
          }
          // end of switch statement
          // end of FSM
          ////
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size -1];
            ref_y = previous_path_y[prev_size -1];            
            double ref_x_prev = previous_path_x[prev_size -2];
            double ref_y_prev = previous_path_y[prev_size -2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s+30.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // transform from global map coordinate to local car coordinate
          for(int i=0;i<ptsx.size();i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x *cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x *sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }
          // create a spline
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          
          ////
          // taking over the rest of last path for next path plan
          ////
          for(int i=0;i<previous_path_x.size(); i++ ){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          
          ////
          // creating new point for completing next path plan, total length of new and taken over path is 50.
          ////

          //divide target_x into N pieces
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;  //initial local x-value = 0
          
          double ref_vel_si = ref_vel/2.24;   // 2.24 mph = 1 meter/sec
          double N = target_dist/0.02/ref_vel_si;
          for(int i=1;i<= 50 - previous_path_x.size();i++){
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
          // transform from local car coordinate to global map coordinate
            x_point = x_ref *cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref *sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
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