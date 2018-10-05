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

    double px_act;      // latency compensated x velocity
    double py_act;      // latency compensated y velocity
    double psi_act;     // latency compensated x velocity
    double v_act;       // latency compensated car velocity
    double cte_act;     // latency compensated cross track error
    double epsi_act;    // latency compensated orientation error

    double N;

public:
    MPCController(MPC mpcInstance);
    json runControlLoop(json &j);
    void transformToCarCoordinates(Eigen::VectorXd &ptsx_vehicle,
                                   Eigen::VectorXd &ptsy_vehicle);
    void compensateForLatency();
    void getInputs( json &j,
                    vector<double> &ptsx,
                    vector<double> &ptsy);
    void getState(Eigen::VectorXd &state);
    void getControlInput(vector<double> &solution);
    void getCTEandEPSI(Eigen::VectorXd &coeffs);

    void setControlOutput(vector<double> &solution,
                          json &msgJson,
                          Eigen::VectorXd ptsx_vehicle,
                          Eigen::VectorXd ptsy_vehicle);
};

#endif
