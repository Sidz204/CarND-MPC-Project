#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <utility> 
#include <tuple>
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
          double previous_a = j[1]["throttle"];
          double previous_delta = j[1]["steering_angle"];
          double v_ref = 60; //reference velocity to apply brakes when above 60


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;


          // Eigen vectors for storing the transformed points
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());

          //convert points from map to car coordinates
          for(int i=0; i<ptsx.size();i++)
          {
            double trans_x = ptsx[i] - px;
            double trans_y = ptsy[i] - py;
            ptsx_car[i] = trans_x * cos(psi) + trans_y * sin(psi);
            ptsy_car[i] = trans_y * cos(psi) - trans_x * sin(psi); 

          }

          //fitting the points into a 3rd degree polynomial
          auto coeffs = polyfit(ptsx_car,ptsy_car,3);

          //since points are in car coordinate system hence x and y are zero
          double target_x = 0.0;
          double target_y = 0.0;
          double target_psi = 0.0;

          //calculate cte and epsi
          double cte = polyeval(coeffs,target_x) - target_y;
          double epsi = target_psi - atan(coeffs[1]); //atan contain the derivative of the polynomial but higher orders are all zero due to transformation into car coordinates 


          double dt = 0.1;
          const double Lf = 2.67;

          //tried adding latency in first time so that car already takes the step but didn't give good results
          /*
          target_x = target_x + v * cos(psi) * dt;
          target_y = target_y + v * sin(psi) * dt;
          target_psi = target_psi + (v/Lf)*(-previous_delta)*dt;
          v = v + previous_a * dt;
          cte = cte + v * sin(epsi) * dt;
          epsi = epsi + (v/Lf)*(-previous_delta)*dt;*/




          Eigen::VectorXd state(6);
          state << target_x, target_y, target_psi, v, cte, epsi; //initial state

          



          //call mpc solve to get next actuations and x ,y
          auto vars = mpc.Solve(state,coeffs);

          

          std::vector<double> x_path;
          std::vector<double> y_path;



          //calculate steering and throttle
          steer_value = vars[0]; 
          throttle_value =  vars[1];

          //slight adjustment when trying with reference velocity of more than 60. P.S: you can try changing v_ref and ref_v values in both cpp files to 100
          if (fabs(steer_value)>0.1 && v_ref>60)
          {
          	throttle_value *=-0.0005; //add a very small component of brakes when the cars takes a sharp turn
          	//steer_value += steer_value*0.00005;
          }


          //std::cout << steer_value << std::endl;

          //adding the predicted trajectory values
          for(int i=4; i<vars.size();i+=2)
          {
            x_path.push_back(vars[i]);
            y_path.push_back(vars[i+1]);
          }


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = x_path;
          vector<double> mpc_y_vals = y_path;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int num_points = 30; //total no of points for plotting on curve
          int increment = 3; //selecting points in gap of 3

          for(int i=3; i<num_points; i++)
          {
            next_x_vals.push_back(i*increment);
            next_y_vals.push_back(polyeval(coeffs,i*increment));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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
