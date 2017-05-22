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
#include "twiddle.h"
#include "cfg.h"

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
double polyeval(const Eigen::VectorXd& coeffs, const double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals,
                        const int order) {
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
  
#if ESTIMATE_PARAMS
  Twiddle twiddle;
  // Initialize the twiddle class.
  //2000,2000,1,5,5,200,10
  std::vector<double> params  = { 2000, 2000, 1, 5, 5, 200, 10 };
  std::vector<double> dparams = { 1000, 1000, 0.5, 2.5, 2.5, 100, 5 };
  twiddle.Init(params, dparams, 0, std::numeric_limits<double>::max(), TWIDDLE_INIT);

  h.onMessage([&mpc, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
#else
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
#endif
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
          vector<double> ptsx       = j[1]["ptsx"];
          vector<double> ptsy       = j[1]["ptsy"];
          const double px           = j[1]["x"];
          const double py           = j[1]["y"];
          const double psi          = j[1]["psi"];
          const double v            = j[1]["speed"];
		  double steer              = j[1]["steering_angle"];
          double throttle           = j[1]["throttle"];

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  
		  // Create Eigen vectors from standard vector
		  Eigen::Map<Eigen::VectorXd> ptsxv(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsyv(&ptsy[0], ptsy.size());
		  
		  // Convert to vehicle's coordinate system
		  const double cosMinPsi   = cos(-psi);
		  const double sinMinPsi   = sin(-psi);
          for (int i = 0; i < ptsxv.size(); ++i) {
            const double x_trans = ptsx[i] - px;
            const double y_trans = ptsy[i] - py;
			
            ptsxv[i] = x_trans * cosMinPsi - y_trans * sinMinPsi;
            ptsyv[i] = x_trans * sinMinPsi + y_trans * cosMinPsi;
          }
		  
		  // From now on, px = 0, py = 0 and psi = 0!
		  const auto coeffs = polyfit(ptsxv,ptsyv,3);
          const double cte  = polyeval(coeffs, 0);
          const double epsi = -atan(coeffs[1]);
		  
#if ESTIMATE_PARAMS
          twiddle.UpdateError(cte);
#endif
		  
		  Eigen::VectorXd state(6);
		  // Account for delay
		  static const double delta_t = 0.1;
		  static const double Lf      = 2.67;
		  
		  const double x_delay = v * delta_t;
          const double y_delay       = 0;
          const double psi_delay     = -v * steer / Lf * delta_t;
          const double v_delay       = v + throttle * delta_t;
          const double cte_delay     = cte + v * sin(epsi) * delta_t;
          const double epsi_delay    = epsi - v * steer /Lf * delta_t;
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
		  
		  MPC_Result result;
#if ESTIMATE_PARAMS
          const bool calcOk = mpc.Solve(state, coeffs, twiddle, result);
#else
		  const bool calcOk = mpc.Solve(state, coeffs, result);
#endif
		  
		  double steer_value, throttle_value;
		  if (calcOk)
		  {
			steer_value    = -result.delta_start;
			throttle_value = result.a_start;
		  }
		  else
		  {
			// Go on with last values
			steer_value    = steer;
			throttle_value = throttle;
		  }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
		  
		  //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  
		  if (calcOk)
		  {
			mpc_x_vals = result.x_path;
			mpc_y_vals = result.y_path;
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
		  for (int i = 0; i < mpc_x_vals.size(); ++i)
		  {
			next_x_vals.push_back(mpc_x_vals[i]);
			next_y_vals.push_back(polyeval(coeffs, mpc_x_vals[i]));
		  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

#if ESTIMATE_PARAMS
		  const bool twiddleDone = twiddle.EstimateTwiddle();
		  if (twiddleDone)
		  {
			// Reset simulator if one twiddle cycle is done
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
		  }
		  else
#endif // ESTIMATE_PARAMS
		  {
			  const auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			  //std::cout << msg << std::endl;
			  // Latency
			  // The purpose is to mimic real driving conditions where
			  // the car does actuate the commands instantly.
			  //
			  // Feel free to play around with this value but should be to drive
			  // around the track with 100ms latency.
			  //
			  // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
			  // SUBMITTING.
			  this_thread::sleep_for(chrono::milliseconds(100));
			  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  }
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
