#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "cfg.h"

using CppAD::AD;

// TODO: Set the timestep length and duration
static const size_t N = 20;
static const double dt = 0.1;

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
static const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
static const double ref_cte  = 0;
static const double ref_epsi = 0;
static const double ref_v    = 100;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
static const size_t x_start = 0;
static const size_t y_start = x_start + N;
static const size_t psi_start = y_start + N;
static const size_t v_start = psi_start + N;
static const size_t cte_start = v_start + N;
static const size_t epsi_start = cte_start + N;
static const size_t delta_start = epsi_start + N;
static const size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
#if ESTIMATE_PARAMS
  Twiddle twiddle;
  FG_eval(const Eigen::VectorXd& coeffs, const Twiddle& aTwiddle) { 
  this->coeffs = coeffs; this->twiddle = aTwiddle; }
#else
  FG_eval(const Eigen::VectorXd& coeffs) { 
  this->coeffs = coeffs; }
#endif

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
	
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	// `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
#if ESTIMATE_PARAMS
      fg[0] += twiddle.getParam(0)*CppAD::pow(vars[cte_start + i] - ref_cte, 2); // 2000
      fg[0] += twiddle.getParam(1)*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); // 2000
      fg[0] += twiddle.getParam(2)*CppAD::pow(vars[v_start + i] - ref_v, 2); // 1
#else
	  fg[0] += 2500*CppAD::pow(vars[cte_start + i] - ref_cte, 2); // 2000
      fg[0] += 2500*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); // 2000
      fg[0] += 1*CppAD::pow(vars[v_start + i] - ref_v, 2); // 1
#endif
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
#if ESTIMATE_PARAMS
      fg[0] += twiddle.getParam(3)*CppAD::pow(vars[delta_start + i], 2); // 5
      fg[0] += twiddle.getParam(4)*CppAD::pow(vars[a_start + i], 2); // 5
#else
	  fg[0] += 10*CppAD::pow(vars[delta_start + i], 2); // 5
      fg[0] += 10*CppAD::pow(vars[a_start + i], 2); // 5
#endif
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
#if ESTIMATE_PARAMS
      fg[0] += twiddle.getParam(5)*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2); // 200
      fg[0] += twiddle.getParam(6)*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2); // 10
#else
	  fg[0] += 300*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2); // 200
      fg[0] += 15*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2); // 10
#endif
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; ++i) {
      // The state at time t+1 .
      const AD<double> x1 = vars[x_start + i + 1];
      const AD<double> y1 = vars[y_start + i + 1];
      const AD<double> psi1 = vars[psi_start + i + 1];
      const AD<double> v1 = vars[v_start + i + 1];
      const AD<double> cte1 = vars[cte_start + i + 1];
      const AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      const AD<double> x0 = vars[x_start + i];
      const AD<double> y0 = vars[y_start + i];
      const AD<double> psi0 = vars[psi_start + i];
      const AD<double> v0 = vars[v_start + i];
      const AD<double> cte0 = vars[cte_start + i];
      const AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      const AD<double> delta0 = vars[delta_start + i];
      const AD<double> a0 = vars[a_start + i];

      const AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      const AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

#if ESTIMATE_PARAMS
bool MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const Twiddle& twiddle, MPC_Result& result) {
#else
bool MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, MPC_Result& result) {
#endif
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // N timesteps == N - 1 actuations
  static const size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  static const size_t n_constraints = N * 6;
  
  const double x    = state[0];
  const double y    = state[1];
  const double psi  = state[2];
  const double v    = state[3];
  const double cte  = state[4];
  const double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
#if ESTIMATE_PARAMS
  FG_eval fg_eval(coeffs, twiddle);
#else
  FG_eval fg_eval(coeffs);
#endif

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
  //options += "String linear_solver mumps\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  const bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  //const auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  // Set predicted driving path
  result.x_path.clear();
  result.y_path.clear();
  for (int i = 0; i < N; ++i)
  {
  	result.x_path.push_back(solution.x[x_start + i]);
  	result.y_path.push_back(solution.x[y_start + i]);
  }
  
  // Set first actuator values
  result.delta_start = solution.x[delta_start];
  result.a_start     = solution.x[a_start];
  return ok; 
}