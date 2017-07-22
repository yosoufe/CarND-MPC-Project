#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen/LU"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void globad2Car_T(const std::vector<double> &pg_x, const std::vector<double> &pg_y,
									double car_x, double car_y, double psi,
									std::vector<double> &pc_x, std::vector<double> &pc_y);

double normalise_angle_diff(double diff);

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

#ifdef WITH_GNUPLOT
FILE *graph_file;
FILE *gp_;
void updateGraphFromFile(double px, double py);
#endif

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

#ifdef WITH_GNUPLOT
	graph_file = std::fopen("graph_Data","w");
	std::fclose(graph_file);
	gp_ = popen("gnuplot -persist" , "w");
#endif

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
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

					v *= 0.44704;

					//updateGraphFromFile(px,py);

					vector<double> c_ptsx;
					vector<double> c_ptsy;

					globad2Car_T(ptsx, ptsy,
											 px, py, psi,
											 c_ptsx, c_ptsy);

					Eigen::Map<Eigen::VectorXd> vecX(&c_ptsx[0],ptsx.size());
					Eigen::Map<Eigen::VectorXd> vecY(&c_ptsy[0],ptsy.size());

					int order = 3;
					Eigen::VectorXd coef = polyfit(vecX,vecY,order);


					double cte = polyeval(coef,0);
					double epsi = - atan(coef[1]);

//					cout << "x: " << rad2deg(px)<<
//									" y: " << rad2deg(py)<<
//									" d_psi: " << rad2deg(d_psi)<<
//									" psi: " << rad2deg(psi)<<
//									" new psi: " << rad2deg(atan(tan(psi)))<<
//									" Before epsi: "<< rad2deg(epsi);
//					//epsi = normalise_angle_diff(epsi);
//					cout << " After epsi: "<< rad2deg(epsi)<< endl;


					Eigen::VectorXd state(6);
					//state << px, py, psi, v, cte, epsi;
					state << 0.0, 0.0, 0.0, v, cte, epsi;

					vector<vector<double>> all_vars;
					vector<double> result = mpc.Solve(state, coef, all_vars);

					double steer_value = -result[0]/deg2rad(25.0);
					double throttle_value = result[1];

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
//					vector<double> mpc_x_vals;
//					vector<double> mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

//					globad2Car_T(all_vars[0], all_vars[1],
//											 px, py, psi,
//											 mpc_x_vals,mpc_y_vals);

					msgJson["mpc_x"] = all_vars[0];
					msgJson["mpc_y"] = all_vars[1];

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

					int i = 0;
					int num = 40;
					for(double x_wp = c_ptsx.front(); i<num ; x_wp+= (c_ptsx.back() - c_ptsx.front())/num){
						i++;
						next_x_vals.push_back(x_wp);
						next_y_vals.push_back(polyeval(coef,x_wp));
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

void globad2Car_T(const std::vector<double> &pg_x, const std::vector<double> &pg_y,
									double car_x, double car_y, double psi,
									std::vector<double> &pc_x, std::vector<double> &pc_y){
	if(pg_x.size() == pg_y.size() && pg_x.size()!=0){
		for (unsigned int i = 0; i<pg_x.size();i++){
			Eigen::VectorXd sr(3);
			sr << pg_x[i],pg_y[i],1;
			Eigen::MatrixXd T_car2glob(3,3);
			T_car2glob << cos(psi),-sin(psi),car_x,
										sin(psi),cos(psi),car_y,
										0,0,1;
			Eigen::VectorXd dst(3);
			dst =  T_car2glob.inverse() * sr;
			pc_x.push_back(dst[0]);
			pc_y.push_back(dst[1]);
		}
	} else {
		std::cout << "incompatible sizes" << std::endl;
	}
}

double normalise_angle_diff(double diff){
	double res = diff;
	if(diff > M_PI/2){
		res = diff - M_PI;
	} else if(diff < -M_PI/2){
		res = diff + M_PI;
	}
	return res;
}

#ifdef WITH_GNUPLOT
void updateGraphFromFile(double px, double py){
	graph_file = std::fopen("graph_Data","a");
	if(graph_file != NULL){
		std::string toWrite = std::to_string(px) + ' '  + std::to_string(py) + '\n'; // error
		std::fwrite(toWrite.c_str(),1,toWrite.length(),graph_file);
		std::fclose(graph_file);
	}

	if (gp_!=NULL){
		fprintf(gp_ , "call '../plotScript.gp'\n");
		std::fflush(gp_);
	}
}
#endif
