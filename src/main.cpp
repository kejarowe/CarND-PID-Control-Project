#include <iostream>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <vector>

#include <uWS/uWS.h>

#include "json.hpp"
#include "PID.h"

#define SIM_TIME 110
#define CRASH_ERROR_VALUE 10000

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void print_gains(std::vector<double> &K)
{
  std::cout << "gains are: ";
  for (auto k : K){
    std::cout << k << ", ";
  }
  std::cout << std::endl;
}

double simulate_car(std::vector<double> K, double run_length)
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.
  pid.Init(K[0],K[1],K[2]);
  time_t start_time;
  time(&start_time);

  //init error
  double error = 0;

  h.onMessage([&pid,&run_length,&start_time,&error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          error += pow(cte,2); //I'm not normalizing by simulation time, so unequal sim lengths will not be handled properly
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value > 1) {
            steer_value = 1;
          } else if (steer_value < -1){
            steer_value = -1;
          }
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          time_t current_time;
          time(&current_time);
          double running_time = difftime(current_time,start_time);
          
          if (run_length > 0 && running_time > run_length)
          { 
            ws.close();
          }
          else if (std::abs(cte) > 5 || (running_time > 2 && speed < 1))
          {
            //the car is off of the track or has crashed 
            error += CRASH_ERROR_VALUE * 45 * (run_length-running_time); //control loop runs at 45Hz
            ws.close();
          } 

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      } 
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    //std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    //ws.close();
    //std::cout << "Disconnected" << std::endl;
    h.getDefaultGroup<uWS::SERVER>().close();
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();

  //reset simulator (only works on OS X)
  system("automator sim_reset.workflow > /dev/null");
  sleep(3);//wait for car to land on track
  return error;
}

std::vector<double> twiddle(double tol = 0.005)
{
  std::vector<double> p = {0,0,0};
  std::vector<double> dp = {1,1,1};
  double best_err = simulate_car(p,SIM_TIME);
  while (dp[0] + dp[1] + dp[2] > tol) {
    for (int i = 0 ; i < 3; i++) {
      p[i] += dp[i];
      double err = simulate_car(p,SIM_TIME);
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
        std::cout << "error is: " << err << " using gains: " << std::endl;
        print_gains(p); 
      }
      else {
        p[i] -= 2*dp[i];
        err = simulate_car(p,SIM_TIME);
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
          std::cout << "error is: " << err << " using gains: " << std::endl;
          print_gains(p);
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
  return p;
}

int main()
{
  std::vector<double> gains;
  //gains = twiddle();
  //std::cout << "best gains:" << std::endl;
  //print_gains(gains);
  gains.push_back(-1.0784); //Kp
  gains.push_back(-0.00694892); //Ki
  gains.push_back(-18.446); //Kd
  simulate_car(gains,-1);
}
