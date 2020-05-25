#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

bool isVehicleInArea(const vector<vector<double>> &vehicles, const double car_s, const double s_lower, const double s_upper)
{
  for (const auto &v : vehicles)
  {
    const auto s{v[5] - car_s};
    if ((s > s_lower) and (s < s_upper))
    {
      return true;
    }
  }

  return false;
}

bool isVehicleInCriticalArea(const vector<vector<double>> &vehicles, const double car_s)
{
  return isVehicleInArea(vehicles, car_s, -10., 15.);
}

bool isVehicleInDangerousArea(const vector<vector<double>> &vehicles, const double car_s)
{
  return isVehicleInArea(vehicles, car_s, -10., 30.);
}

bool isVehicleTTCBelowThreshold(const vector<vector<double>> &vehicles, const double car_s, const double car_v, const double ttc_min)
{
  for (const auto &v : vehicles)
  {
    const auto v_rel{sqrt(v[3] * v[3] + v[4] * v[4]) - car_v};
    const auto s{v[5] - car_s};
    if (abs(s) > 1E-6)
    {
      const auto ttc{-s / v_rel};
      if (1. / ttc > 1. / ttc_min)
      {
        return true;
      }
    }
  }

  return false;
}

bool isVehicleTTCCritical(const vector<vector<double>> &vehicles, const double car_s, const double car_v)
{
  return isVehicleTTCBelowThreshold(vehicles, car_s, car_v, 5.);
}

bool isLaneInRange(const int lane)
{
  return lane >= 0 and lane <= 2;
}

bool isLaneValidForLaneChange(const int lane, const vector<vector<double>> &vehicles, const double car_s, const double car_v)
{
  return isLaneInRange(lane) and !isVehicleInDangerousArea(vehicles, car_s) and !isVehicleTTCCritical(vehicles, car_s, car_v);
}

double getVehicleTTC(const vector<double> &vehicle, const double car_s, const double car_v)
{
  if (vehicle.size() == 0)
  {
    return 1E9;
  }
  const double v_rel{sqrt(vehicle[3] * vehicle[3] + vehicle[4] * vehicle[4]) - car_v};
  const double s_rel{vehicle[5] - car_s};
  if (v_rel == 0)
  {
    return 1E9;
  }
  return -s_rel / v_rel;
}

vector<vector<vector<double>>> getOrderedVehicles(const vector<vector<double>> &unordered_vehicles)
{
  vector<vector<vector<double>>> vehicles(3, vector<vector<double>>());
  for (const auto &v : unordered_vehicles)
  {
    const double d{v[6]};
    const auto lane{static_cast<int>(round((d - 2.) / 4.))};
    if (lane >= 0)
    {
      vehicles[lane].push_back(v);
    }
  }

  return vehicles;
}

vector<double> getVehicleInFront(const vector<vector<double>> &vehicles, const double car_s)
{
  if (0 == vehicles.size())
  {
    return {};
  }
  auto min_s{99999.};
  auto min_veh{vehicles[0]};
  auto vehicle_found{false};
  for (const auto &veh : vehicles)
  {
    const auto s_diff{veh[5] - car_s};
    if (s_diff > 0. and s_diff < min_s)
    {
      min_s = s_diff;
      min_veh = veh;
      vehicle_found = true;
    }
  }
  if (vehicle_found)
  {
    return min_veh;
  }
  else
  {
    return {};
  }
}

int getFreeLaneForDoubleLaneChange(const vector<vector<vector<double>>> &vehicles, const int lane, const double car_s, const double car_v)
{
  if (lane == 0 and
      !isVehicleInCriticalArea(vehicles[1], car_s) and
      !isVehicleTTCCritical(vehicles[1], car_s, car_v) and
      !isVehicleInDangerousArea(vehicles[2], car_s) and
      !isVehicleTTCCritical(vehicles[2], car_s, car_v))
  {
    return 2;
  }
  else if (lane == 2 and
           !isVehicleInCriticalArea(vehicles[1], car_s) and
           !isVehicleTTCCritical(vehicles[1], car_s, car_v) and
           !isVehicleInDangerousArea(vehicles[0], car_s) and
           !isVehicleTTCCritical(vehicles[0], car_s, car_v))
  {
    return 0;
  }

  return lane;
}

int getFreeLaneForLaneChange(const vector<vector<vector<double>>> &vehicles, const int lane, const double car_s, const double car_v)
{
  int lane_to_switch{lane};

  const int lane_left{lane - 1};
  const int lane_right{lane + 1};

  const auto lane_left_valid{isLaneValidForLaneChange(lane_left, vehicles[lane_left], car_s, car_v)};
  const auto lane_right_valid{isLaneValidForLaneChange(lane_right, vehicles[lane_right], car_s, car_v)};

  if (lane_left_valid and lane_right_valid)
  {
    const double ttc_left{getVehicleTTC(getVehicleInFront(vehicles[lane_left], car_s), car_s, car_v)};
    const double ttc_right{getVehicleTTC(getVehicleInFront(vehicles[lane_right], car_s), car_s, car_v)};

    lane_to_switch = (ttc_left >= ttc_right) ? lane_left : lane_right;
  }
  else if (lane_left_valid)
  {
    lane_to_switch = lane_left;
  }
  else if (lane_right_valid)
  {
    lane_to_switch = lane_right;
  }
  else
  {
    lane_to_switch = getFreeLaneForDoubleLaneChange(vehicles, lane, car_s, car_v);
  }

  return lane_to_switch;
}

std::pair<int, double> getReference(const vector<vector<vector<double>>> &vehicles, const int lane, const double car_s, const double car_v)
{
  const auto lane_to_switch{getFreeLaneForLaneChange(vehicles, lane, car_s, car_v)};
  const auto vehicle_in_front{getVehicleInFront(vehicles[lane], car_s)};

  int lane_ref{lane};
  double v_ref{0.};

  if (vehicle_in_front.size())
  {
    const double v{sqrt(vehicle_in_front[3] * vehicle_in_front[3] + vehicle_in_front[4] * vehicle_in_front[4])};
    const double s_rel{vehicle_in_front[5] - car_s};
    const double ttc{getVehicleTTC(vehicle_in_front, car_s, car_v)};

    if (1. / ttc > 1. / 5. or s_rel < 30.)
    {
      if (lane_to_switch != lane)
      {
        lane_ref = lane_to_switch;
      }
      else
      {
        v_ref = ((s_rel < 20.) ? .75 : 1.) * v;
      }
      if (1. / ttc > 1. / .5)
      {
        v_ref = 0.;
      }
    }
    else
    {
      v_ref = .98 * 50. / 2.24;
    }
  }
  else
  {
    v_ref = .98 * 50. / 2.24;
  }

  return {lane_ref, v_ref};
}

double getControlVelocityFromReference(const double ref_v, const double car_v)
{
  const auto diff_vel{ref_v - car_v};
  if (fabs(diff_vel) > .7)
  {
    if (diff_vel > 0.)
    {
      return car_v + .7;
    }
    else
    {
      return car_v - .7;
    }
  }

  return car_v;
}

#endif // HELPERS_H
