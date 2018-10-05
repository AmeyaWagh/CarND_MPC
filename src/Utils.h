#ifndef UTILS_H
#define UTILS_H
#include "json.hpp"
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals,
                        Eigen::VectorXd yvals,
                        int order);


// Higher level MPC controller class
class MPCController{
private:
    MPC mpc;
    double px;
    double py;
    double psi;
    double v;

    double steer_value;
    double throttle_value;

    double delta;
    double accel;

    double cte;
    double epsi;
    double latency;

    double px_act;
    double py_act;
    double psi_act;
    double v_act;
    double cte_act;
    double epsi_act;

public:
    MPCController(MPC mpcInstance);
    json runControlLoop(json &j);
};

#endif
