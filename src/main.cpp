#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;



            double delta = j[1]["steering_angle"];
            double accel = j[1]["throttle"];

            // transform coordinates to car
            Eigen::VectorXd ptsx_vehicle = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
            Eigen::VectorXd ptsy_vehicle = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

            for(int i = 0; i < ptsx_vehicle.size(); i++) {
              double x = ptsx_vehicle[i] - px;
              double y = ptsy_vehicle[i] - py;
              ptsx_vehicle[i] = x * cos(psi) + y * sin(psi);
              ptsy_vehicle[i] = - x * sin(psi) + y * cos(psi);
            }

            // 3 order polyfit for waypoints
          auto coefficients = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

          // get Cross Track Error CTE (at x=0)
          double cte = polyeval(coefficients, 0);

          // Orientation error, -atan(b1 + b2*x, b3* x^2) (as car is at x=0)
          double epsi = -atan(coefficients[1]);

          // to account for the latency (100ms predict the state of the car after this)
          double latency = 100.0/1000.0;

          double px_act = v * latency;
          double py_act = 0;
          double psi_act = - v * delta * latency / 2.67;
          double v_act = v + accel * latency;
          double cte_act = cte + v * sin(epsi) * latency;
          double epsi_act = epsi + psi_act;

          // state vector
          Eigen::VectorXd state(6);
          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;

          std::cout << "************** Sending trajectories to MPC Solver *****************************\n";
          // solve the trajectory
          auto solution = mpc.Solve(state, coefficients);

          std::cout << "************** Obtained solution from solver ***********************************\n";
          // get the steering and throttle values
          steer_value = -solution[0]/ deg2rad(25);
          throttle_value = solution[1];
          auto N = solution[2];
          std::cout << "************** loaded solution ***********************************\n";

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          std::cout << "************** displaying MPC trajs ***********************************\n";
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;


          std::cout << "************** loaded mpc_x_vals and mpc_y_vals ***********************************\n";
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for(int i = 0; i < N-1; i++) {
              mpc_x_vals.push_back(solution[i+3]);
              mpc_y_vals.push_back(solution[N + i+3]);
            }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          std::cout << "************** Displaying vals ***********************************\n";
          //Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_vehicle.size());
          vector<double> next_y_vals(ptsx_vehicle.size());

          std::cout << "************** loaded next_x_vals and next_y_vals ***********************************\n";
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for(int i = 0; i < ptsx_vehicle.size(); i++) {
              next_x_vals[i] = ptsx_vehicle[i];
              next_y_vals[i] = ptsy_vehicle[i];
            }

          std::cout << "************** populating json ***********************************\n";
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          std::cout << "************** printing json ***********************************\n";
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::cout << "************** Sending data ***********************************\n";
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          std::cout << "************** DONE ***********************************\n";
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
