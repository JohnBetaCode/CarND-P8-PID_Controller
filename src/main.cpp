#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double calculate_throttle(double steering) {
  double new_throttle;
  new_throttle = (1 - std::abs(steering)) * 0.5 + 0.2;
  return new_throttle; 
  }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  std::cout << "Starting PID program" << std::endl;

  // Create PID variable
  PID pid_steering; // PID for car's steering angle
  PID pid_speed; // PID for car's speed

  std::ofstream steering_csv;
  steering_csv.open ("pid_steering.csv");
  steering_csv << "json_speed, json_angle, json_cte, throttle, steer_pid, steer_p_error, steer_i_error, steer_d_error\n";

  // Inizializate of  PID controller (Kp_, Ki_, Kd_)
  pid_steering.Init(0.2, 0.01, 3.0); 
  pid_speed.Init(0.2, 0.01, 3.0); 
  std::cout << "PID Controllers initialized" << std::endl;

  // Setting speed reference
  double desired_speed = 30.0;

  h.onMessage([&pid_steering, &pid_speed, &steering_csv, &desired_speed](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          
          // j[1] is the data JSON object
          double json_steer_cte = std::stod(j[1]["cte"].get<string>());  // car's distance from the lane center
          double json_speed = std::stod(j[1]["speed"].get<string>()); // Current car's speed
          double json_angle = std::stod(j[1]["steering_angle"].get<string>()); // Current car's Steering angle
          double steer_value;

          // DEBUG
          std::cout << "\n| JSON\t| cte: " << json_steer_cte // car's distance from the lane center
                    << "\t| speed: "    << json_angle  // Current car's speed
                    << "\t| steering: " << json_speed  // Current car's Steering angle
                    << "\t|" << std::endl;

          //*******************************************************************          
          // Update error values with cte
          pid_steering.UpdateError(json_steer_cte);

          // Calculate steering value (if reasonable error, returns between [-1, 1])
          steer_value = pid_steering.TotalError();

          // DEBUG
          double steer_p_error = pid_steering.get_p_error();
          double steer_i_error = pid_steering.get_i_error();
          double steer_d_error = pid_steering.get_d_error();
          std::cout << "| PID \t| steering: " << steer_value
                    << "\t| p_error: " << steer_p_error
                    << "\t| i_error: " << steer_i_error
                    << "\t| d_error: " << steer_d_error
                    << "\t|" << std::endl;

          // Calculate new throttle value
          double throttle = calculate_throttle(steer_value);

          // Read values to csv file to plot them later
          steering_csv 
            << json_speed << ","
            << json_angle << ","
            << json_steer_cte << ","
            << throttle << ","
            << steer_value << ","
            << steer_p_error << ","
            << steer_i_error << ","
            << steer_d_error << ","
            <<"\n";
          
          //*******************************************************************
          // Send steering and throttle values to simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; 

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        steering_csv.close();
      }
    }  // end websocket message if
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