#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const int latency_ms = 100;
const double latency_s = 0.1;

int main()
{
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					   uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//std::cout << sdata << std::endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
		{
			string s = hasData(sdata);
			if (s != "")
			{
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry")
				{
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];
					double steering_angle = j[1]["steering_angle"];
					double throttle = j[1]["throttle"];

					vector<double> waypts_x;
					vector<double> waypts_y;

					for (size_t i = 0; i < ptsx.size(); i++)
					{
						double dx = ptsx[i] - px;
						double dy = ptsy[i] - py;
						waypts_x.emplace_back(dx * cos(-psi) - dy * sin(-psi));
						waypts_y.emplace_back(dx * sin(-psi) + dy * cos(-psi));
					}

					Eigen::Map<Eigen::VectorXd> eig_waypts_x(waypts_x.data(), waypts_x.size());
					Eigen::Map<Eigen::VectorXd> eig_waypts_y(waypts_y.data(), waypts_y.size());

					auto coeffs = polyfit(eig_waypts_x, eig_waypts_y, 3);
					auto cte = polyeval(coeffs, 0);
					auto epsi = -atan(coeffs[1]);


					Eigen::VectorXd state(6);
					state << 0, 0, 0, v, cte, epsi;
					state(0) += v * cos(state(2)) * latency_s;
					state(1) += v * sin(state(2)) * latency_s;
					state(2) -= v * steering_angle * latency_s / Lf;
					state(3) += throttle * latency_s;
					state(4) += v * sin(state(5)) * latency_s;
					state(5) -= v * atan(coeffs[1]) * latency_s / Lf;


					auto solution = mpc.Solve(state, coeffs);
					auto steer_value = solution[0] / deg2rad(25);
					auto throttle_value = solution[1];

					json msgJson;
					// NOTE: Remember to divide by deg2rad(25) before you send the
					//   steering value back. Otherwise the values will be in between
					//   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle_value;

					//std::cout << "Steer: " << steer_value << ", throttle: " << throttle << std::endl;

					// Display the MPC predicted trajectory
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;

					for (int i = 2; i < solution.size(); i++)
					{
						if (i % 2 == 0)
						{
							mpc_x_vals.emplace_back(solution[i]);
						}
						else
						{
							mpc_y_vals.emplace_back(solution[i]);
						}
					}

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

					// Display the waypoints/reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					for (double i = 0; i < 80; i += 2)
					{
						next_x_vals.push_back(i);
						next_y_vals.push_back(polyeval(coeffs, i));
					}

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					// Latency
					// The purpose is to mimic real driving conditions where
					//   the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					//   around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
					std::this_thread::sleep_for(std::chrono::milliseconds(latency_ms));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				} // end "telemetry" if
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		} // end websocket if
	});   // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}