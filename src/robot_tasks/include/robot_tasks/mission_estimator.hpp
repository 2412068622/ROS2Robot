#pragma once

#include "robot_tasks/mission_profile.hpp"

#include <string>

namespace robot_tasks {

struct MissionCostEstimate {
  int waypoint_count = 0;
  double distance_m = 0.0;
  double eta_sec = 0.0;
  double battery_drop_v = 0.0;
  double projected_battery_voltage = 0.0;
  bool battery_sufficient = true;
  std::string distance_source = "waypoint_polyline";
};

double EstimateMissionPathDistance(const MissionProfile& profile);
MissionCostEstimate EstimateMissionCost(
    const MissionProfile& profile, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage);
MissionCostEstimate EstimateDistanceCost(
    double distance_m, int waypoint_count, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage);
MissionCostEstimate EstimateDistanceCost(
    double distance_m, int waypoint_count, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage,
    const std::string& distance_source);

}  // namespace robot_tasks
