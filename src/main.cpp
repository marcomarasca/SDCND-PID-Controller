#include <math.h>
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "PID.h"
#include "Tuner.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double clamp_steering(double n) { return n < -1 ? -1 : (n > 1 ? 1 : n); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER> &ws) {
  std::cout << "Resetting simulator" << std::endl;
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void runSimulation(double Kp, double Ki, double Kd, unsigned int max_steps) {
  uWS::Hub h;

  PID steering_pid;

  std::vector<double> params = {Kp, Kd};
  std::vector<double> params_delta = {0.1, 0.5};

  Tuner tuner = {params, params_delta, max_steps};

  if (tuner.Enabled()) {
    std::cout << "Tuning ENABLED" << std::endl;
    tuner.PrintParams();
    tuner.PrintParamsDelta();
  }

  // Initializes the controller coefficients
  steering_pid.Init(Kp, Ki, Kd);

  std::ostringstream oss;

  oss << "cte_out_" << Kp << "_" << Ki << "_" << Kd << ".txt";

  std::string file_name = oss.str();

  std::ofstream file_out(file_name);

  if (!file_out.is_open()) {
    std::cout << "Could not open file" << file_name << " for writing" << std::endl;
    exit(EXIT_FAILURE);
  }

  h.onMessage([&steering_pid, &file_out, &tuner, &max_steps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                             uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          if (tuner.Enabled()) {
            std::vector<double> tuned_params = tuner.Tune(cte);

            steering_pid.Init(tuned_params[0], 0, tuned_params[1]);

            if (tuner.IsResetCycle()) {
              reset_simulator(ws);
              return;
            }
          }

          // Updates the controller errors
          steering_pid.UpdateError(cte);

          // Gets the total error and uses it as the steering angle
          double steer_value = steering_pid.TotalError();
          // Clamp the value between 1 and -1
          steer_value = clamp_steering(steer_value);

          // Set throttle value according to steering value, the more the angle the less the throttle.
          // Min throttle 0.3, max throttle 0.5
          double throttle = (1 - fabs(steer_value)) * 0.2 + 0.3;

          // DEBUG
          if (!tuner.Enabled()) {
            std::cout << "Current Speed: " << speed << ", Current Steering Angle: " << angle << std::endl;
            std::cout << "CTE: " << cte << ", Steering Value: " << steer_value << " Throttle: " << throttle
                      << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;

          // Writes output to file
          file_out << speed << "\t";
          file_out << angle << "\t";
          file_out << cte << "\t";
          file_out << steer_value << "\t";
          file_out << throttle << std::endl;
          file_out.flush();

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection(
      [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h, &file_out](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
    if (file_out.is_open()) {
      file_out.close();
    }
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    if (file_out.is_open()) {
      file_out.close();
    }
    exit(EXIT_FAILURE);
  }

  h.run();
}

int main(int argc, char *argv[]) {
  double Kp = 0.2;
  double Ki = 0.0;
  double Kd = 5.2;

  unsigned int max_steps = 0; // 4500 for entire lap

  if (argc > 1) {
    if (argc < 4) {
      std::cerr << "Number of required arguments does not match: requires 3, got: " << argc << std::endl;
      exit(EXIT_FAILURE);
    }

    std::istringstream iss;

    iss.str(argv[1]);
    if (!(iss >> Kp)) {
      std::cout << "Could not read Kp coefficient, using 0" << std::endl;
    }
    iss.clear();
    iss.str(argv[2]);
    if (!(iss >> Ki)) {
      std::cout << "Could not read Ki coefficient, using 0" << std::endl;
    }
    iss.clear();
    iss.str(argv[3]);
    if (!(iss >> Kd)) {
      std::cout << "Could not read Kd coefficient, using 0" << std::endl;
    }

    if (argc > 4) {
      iss.clear();
      iss.str(argv[4]);
      if (!(iss >> max_steps)) {
        std::cout << "Could not read max_steps, Tuning DISABLED" << std::endl;
      }
    }
  }

  std::cout << "Using PID cofficients: " << Kp << " " << Ki << " " << Kd << std::endl;

  runSimulation(Kp, Ki, Kd, max_steps);
}
