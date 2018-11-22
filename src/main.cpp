#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "behavior_planner.h"
#include "trajectory_generator.h"
#include "json.hpp"

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

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";
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

  BehaviorPlanner planner = BehaviorPlanner();
  
#ifdef UWS_VCPKG
  // code fixed for latest uWebSockets
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&planner](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
                     uWS::OpCode opCode) {
#else
  // leave original code here
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	  uWS::OpCode opCode) {
#endif
	  
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
			vector<double> previous_path_x = j[1]["previous_path_x"];
			vector<double> previous_path_y = j[1]["previous_path_y"];
			// Previous path's end s and d values 
			double end_path_s = j[1]["end_path_s"];
			double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			auto sensor_fusion = j[1]["sensor_fusion"];

			// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds			

			// call the behavior planner to choose the target lane and velocity
			planner.choose_next_state(car_s, car_d, car_speed, sensor_fusion);
			int target_velocity = planner.v;
			int target_lane = planner.lane;
			
			// call the trajector generator to calculate the next set of path points			
			vector<vector<double>> next_vals = TrajectoryGenerator::generate_next_trajectory(car_x, car_y,
				car_s, car_yaw, previous_path_x,
				previous_path_y, map_waypoints_s,
				map_waypoints_x, map_waypoints_y,
				target_lane, target_velocity);
		    
			// assign the next set of x and y path values
			vector<double> next_x_vals = next_vals[0];
			vector<double> next_y_vals = next_vals[1];
			
			json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
#ifdef UWS_VCPKG
          	ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }
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
#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	  std::cout << "Connected!!!" << std::endl;
  });
#endif

#ifdef UWS_VCPKG
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	  char *message, size_t length) {
	  ws.close();
	  std::cout << "Disconnected" << std::endl;
  });
#endif

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

