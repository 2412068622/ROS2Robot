#include "robot_tasks/mission_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace robot_tasks {

double EstimateMissionPathDistance(const MissionProfile& profile) {
  double distance_m = 0.0;
  for (std::size_t i = 1; i < profile.waypoints.size(); ++i) {
    const auto& previous = profile.waypoints[i - 1];
    const auto& current = profile.waypoints[i];
    distance_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return distance_m;
}

MissionCostEstimate EstimateMissionCost(
    const MissionProfile& profile, const double nominal_speed_mps, const double battery_voltage,
    const double battery_drop_per_meter, const double minimum_battery_voltage) {
  const double distance_m = EstimateMissionPathDistance(profile);
  return EstimateDistanceCost(
      distance_m, static_cast<int>(profile.waypoints.size()), nominal_speed_mps, battery_voltage,
      battery_drop_per_meter, minimum_battery_voltage, "waypoint_polyline");
}

MissionCostEstimate EstimateDistanceCost(
    const double distance_m, const int waypoint_count, const double nominal_speed_mps,
    const double battery_voltage, const double battery_drop_per_meter,
    const double minimum_battery_voltage) {
  return EstimateDistanceCost(
      distance_m, waypoint_count, nominal_speed_mps, battery_voltage, battery_drop_per_meter,
      minimum_battery_voltage, "distance_override");
}

MissionCostEstimate EstimateDistanceCost(
    const double distance_m, const int waypoint_count, const double nominal_speed_mps,
    const double battery_voltage, const double battery_drop_per_meter,
    const double minimum_battery_voltage, const std::string& distance_source) {
  MissionCostEstimate estimate;
  estimate.waypoint_count = waypoint_count;
  estimate.distance_m = std::max(distance_m, 0.0);
  estimate.distance_source = distance_source.empty() ? "distance_override" : distance_source;

  const double safe_speed = std::max(nominal_speed_mps, 0.01);
  const double safe_drop_per_meter = std::max(battery_drop_per_meter, 0.0);
  estimate.eta_sec = estimate.distance_m / safe_speed;
  estimate.battery_drop_v = estimate.distance_m * safe_drop_per_meter;
  estimate.projected_battery_voltage = battery_voltage - estimate.battery_drop_v;
  estimate.battery_sufficient =
      battery_voltage <= 0.0 || estimate.projected_battery_voltage >= minimum_battery_voltage;
  return estimate;
}

}  // namespace robot_tasks
