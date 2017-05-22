#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "twiddle.h"
#include "cfg.h"

using namespace std;

struct MPC_Result
{
  // Return first actuator values
  double delta_start;
  double a_start;
  
  // Return predicted driving path
   vector<double> x_path;
   vector<double> y_path;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions and the predicted driving path.
#if ESTIMATE_PARAMS
  bool Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const Twiddle& twiddle, MPC_Result& result);
#else
  bool Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, MPC_Result& result);
#endif
};

#endif /* MPC_H */
