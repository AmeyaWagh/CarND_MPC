#include "Utils.h"
#include "json.hpp"
#include <math.h>
#include <iostream>

using json = nlohmann::json;
using namespace std;


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



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



double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


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


// MpcController

MPCController::MPCController(MPC mpcInstance):mpc(mpcInstance){

    // to account for the latency 100ms offset
    latency = 100.0/1000.0;
}

json
MPCController::runControlLoop(json &j)
{

    vector<double> ptsx = j[1]["ptsx"];
    vector<double> ptsy = j[1]["ptsy"];
    px = j[1]["x"];
    py = j[1]["y"];
    psi = j[1]["psi"];
    v = j[1]["speed"];

    delta = j[1]["steering_angle"];
    accel = j[1]["throttle"];

    // transform coordinates to car
    Eigen::VectorXd ptsx_vehicle = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
    Eigen::VectorXd ptsy_vehicle = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

    for(int i = 0; i < ptsx_vehicle.size(); i++) {
        double x = ptsx_vehicle[i] - px;
        double y = ptsy_vehicle[i] - py;
        ptsx_vehicle[i] = x * cos(psi) + y * sin(psi);
        ptsy_vehicle[i] = - x * sin(psi) + y * cos(psi);
    }

    // 3rd order polyfit
    auto coefficients = polyfit(ptsx_vehicle, ptsy_vehicle, 3);


    cte = polyeval(coefficients, 0);

    // Orientation error, -atan(b1 + b2*x, b3* x^2) (as car is at x=0)
    epsi = -atan(coefficients[1]);


    px_act = v * latency;
    py_act = 0;
    psi_act = - v * delta * latency / 2.67;
    v_act = v + accel * latency;
    cte_act = cte + v * sin(epsi) * latency;
    epsi_act = epsi + psi_act;

    // state vector
    Eigen::VectorXd state(6);
    state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;


    // solve the trajectory
    auto solution = mpc.Solve(state, coefficients);


    // get the steering and throttle values
    static const double steer_value_deno = deg2rad(25);
    steer_value = -solution[0]/ steer_value_deno;
    throttle_value = solution[1];
    auto N = solution[2];


    json msgJson;
    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle_value;


    //Display the MPC predicted trajectory
    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;



    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Green line
    for(int i = 0; i < N-1; i++) {
        mpc_x_vals.push_back(solution[i+3]);
        mpc_y_vals.push_back(solution[N + i+3]);
    }

    msgJson["mpc_x"] = mpc_x_vals;
    msgJson["mpc_y"] = mpc_y_vals;


    //Display the waypoints/reference line
    vector<double> next_x_vals(ptsx_vehicle.size());
    vector<double> next_y_vals(ptsx_vehicle.size());


    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
    // the points in the simulator are connected by a Yellow line
    for(int i = 0; i < ptsx_vehicle.size(); i++) {
        next_x_vals[i] = ptsx_vehicle[i];
        next_y_vals[i] = ptsy_vehicle[i];
    }


    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    return msgJson;
}
