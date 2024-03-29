#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // desired lane and velocity are tracked and updated
  int lane{1};
  auto velocity{.98 * 50 / 2.24};

  h.onMessage([&map_waypoints_x,
               &map_waypoints_y,
               &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy,
               &lane,
               &velocity](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                          uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    auto max_v{50. / 2.24};
    auto dT{0.02};
    auto N{50};

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(static_cast<double>(j[1]["yaw"]));   // convert directly to radians
          double car_speed = static_cast<double>(j[1]["speed"]) / 2.24; // convert directly to mps

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // get 3 lists of objects. each one contains the objects assigned to one of the lanes
          const auto vehicles{getOrderedVehicles(sensor_fusion)};

          // core of the planning algorithm. reference is updated
          const auto reference{getReference(vehicles, lane, car_s, car_speed)};
          lane = std::get<0>(reference);
          velocity = std::get<1>(reference);

          // generate the velocity that the car will follow in order to reach desired velocity
          const auto ref_vel{getControlVelocityFromReference(velocity, car_speed)};

          ///////////////////////////////////////////////////////////
          //                                                       //
          // Trajectory generator                                  //
          // Section taken from David's and Aaron's Q&A Session    //
          //                                                       //
          ///////////////////////////////////////////////////////////

          const auto prev_size{previous_path_x.size()};

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x{car_x};
          double ref_y{car_y};
          double ref_yaw{car_yaw};

          if (prev_size < 2)
          {
            const auto prev_car_x{car_x - cos(car_yaw)};
            const auto prev_car_y{car_y - sin(car_yaw)};

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            const double ref_x_prev{previous_path_x[prev_size - 2]};
            const double ref_y_prev{previous_path_y[prev_size - 2]};
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          const vector<double> next_wp0{getXY(car_s + (.25 + .75 * car_speed / max_v) * 60., 2. + 4. * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};
          const vector<double> next_wp1{getXY(car_s + (.25 + .75 * car_speed / max_v) * 120., 2. + 4. * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};
          const vector<double> next_wp2{getXY(car_s + (.25 + .75 * car_speed / max_v) * 180., 2. + 4. * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i{0}; i < ptsx.size(); ++i)
          {
            const auto shift_x{ptsx[i] - ref_x};
            const auto shift_y{ptsy[i] - ref_y};

            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for (int i{0}; i < prev_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          const auto target_x{30.};
          const auto target_y{s(target_x)};
          const auto target_dist{sqrt(target_x * target_x + target_y * target_y)};

          auto x_add_on{0.};

          for (int i{0}; i < N - prev_size; ++i)
          {
            const auto N{target_dist / (dT * ref_vel)};
            auto x_point{x_add_on + target_x / N};
            auto y_point{s(x_point)};

            x_add_on = x_point;

            const auto x_ref{x_point};
            const auto y_ref{y_point};

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          // END of Trajectory generator

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
  }); // end h.onMessage

  h.onConnection([&h, &lane](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    lane = 1;
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
