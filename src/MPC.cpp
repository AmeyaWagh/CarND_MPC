#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 15;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N -1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
//     fg[0]=CppAD::pow(0,1);
      fg[0]=0;
//      fg[0]=CppAD::AD::
      ////////////////  Cost Function   //////////////////
    // State: [x,y,psi,v,cte,epsi]
    // Cost based on Reference State; for Trajectory, set cost to 0 (i.e. follow Reference)
    for (size_t t = 0; t < N; t++) {
        fg[0] +=  W_V * CppAD::pow(vars[cte_start + t] - REFERENCE_CTE, 2);   // Cross Track Error  ( 0 reference track error)
        fg[0] +=  W_EPSI * CppAD::pow(vars[epsi_start + t] - REFERENCE_ERROR, 2); // Orientation error ( 0 reference error)
        fg[0] +=  W_CTE * CppAD::pow(vars[v_start + t] - REFERENCE_VELOCITY, 2);       // Velocity error (75 reference velocity)
    }

    // Minimise the use of Actuators, i.e. minimize the change rate ([δ,a])
    // Add Scaling factor to smooth out
    for (size_t t = 0; t < N - 1; t++) {
        fg[0] +=  W_DELTA * CppAD::pow(vars[delta_start + t], 2); // steering angle
        fg[0] +=  W_ACC   * CppAD::pow(vars[a_start + t], 2);     // acceleration (throttle)
    }

    // Minimize the value gap between sequential actuations
    // Add Scaling factor to smooth out
    for (size_t t = 0; t < N - 2; t++) {
        fg[0] +=  W_DELTA_DOT * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] +=  W_ACC_DOT   * CppAD::pow(vars[a_start + t + 1]  - vars[a_start + t], 2);
    }

    ////////////// Setup constraints //////////////////
    // Initial Constraints
    // initialize the model to the initial state.

    // as fg[0] is used for cost value, we use 1
    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];

    // remaining Constraints
    for (size_t t = 1; t < N ; t++) {
        // State at t + 1
        AD<double>  x1    = vars[x_start + t];
        AD<double>  y1    = vars[y_start + t];
        AD<double>  psi1  = vars[psi_start + t];
        AD<double>  v1    = vars[v_start + t];
        AD<double>  cte1  = vars[cte_start + t];
        AD<double>  epsi1 = vars[epsi_start + t];

        // State at t
        AD<double>  x0    = vars[x_start + t - 1];
        AD<double>  y0    = vars[y_start + t - 1];
        AD<double>  psi0  = vars[psi_start + t - 1];
        AD<double>  v0    = vars[v_start + t - 1];
        AD<double>  cte0  = vars[cte_start + t - 1];
        AD<double>  epsi0 = vars[epsi_start + t - 1];

        // at t
        AD<double>  delta0  = vars[delta_start + t - 1];
        AD<double>  a0      = vars[a_start + t - 1];

        // polynomial of degree 3
        AD<double>  f0      = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3]*x0 * x0 * x0;
        AD<double>  psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0  + 3*coeffs[3]*x0*x0);


        // Constrain to 0
        fg[1 + x_start + t]   = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t]   = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1  - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start + t]   = v1    - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t]= epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
//  size_t i;

  typedef CPPAD_TESTVECTOR(double) Dvector;


  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
//  size_t n_vars = 0;
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
//  size_t n_constraints = 0;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.


  Dvector vars(n_vars);

  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

    vars[x_start]   = state[0];
    vars[y_start]   = state[1];
    vars[psi_start] = state[2];
    vars[v_start]   = state[3];
    vars[cte_start] = state[4];
    vars[epsi_start]= state[5];


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set non-actuator upper and lower-limits
  for (size_t i = 0; i < delta_start; i++) {
      vars_lowerbound[i]  = -1.0e19;
      vars_upperbound[i]  = 1.0e19;
  }

  // steering angle constrains
  for (size_t i = delta_start; i < a_start; i++) {
      vars_lowerbound[i]  = -STEERING_CONSTRAINT;
      vars_upperbound[i]  = STEERING_CONSTRAINT;
  }


  // 4d - acceleration constrains: +/- 1.0
  for (size_t i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -1.0;
      vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // lowerbound
  constraints_lowerbound[x_start]     = state[0];
  constraints_lowerbound[y_start]     = state[1];
  constraints_lowerbound[psi_start]   = state[2];
  constraints_lowerbound[v_start]     = state[3];
  constraints_lowerbound[cte_start]   = state[4];
  constraints_lowerbound[epsi_start]  = state[5];


  // uppwerbound
  constraints_upperbound[x_start]     = state[0];
  constraints_upperbound[y_start]     = state[1];
  constraints_upperbound[psi_start]   = state[2];
  constraints_upperbound[v_start]     = state[3];
  constraints_upperbound[cte_start]   = state[4];
  constraints_upperbound[epsi_start]  = state[5];



  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result = {solution.x[delta_start], solution.x[a_start], (double)N};

      for(size_t t = 0; t < N; t++) {
          result.push_back(solution.x[x_start + t+1]);
      }

      for(size_t t = 0; t < N; t++) {
          result.push_back(solution.x[y_start + t+1]);
      }
  return result;
}
