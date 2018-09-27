#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// Scale factors for Cost
#define STEERING_CONSTRAINT 0.436332f     // 25 degrees in radians
#define REFERENCE_VELOCITY 40
#define REFERENCE_ERROR 0
#define REFERENCE_CTE 0
#define W_V  1.0
#define W_EPSI  10.0
#define W_CTE 1.0
#define W_DELTA  500.0
#define W_ACC 1.0
#define W_DELTA_DOT 25.0
#define W_ACC_DOT 1.0


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
