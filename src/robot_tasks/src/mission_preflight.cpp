#include "robot_tasks/mission_preflight.hpp"

#include "robot_navigation/zone_catalog.hpp"

#include <exception>
#include <sstream>

namespace robot_tasks {
namespace {

MissionPreflightResult Reject(const std::string& message, const MissionCostEstimate& cost = {}) {
  MissionPreflightResult result;
  result.allowed = false;
  result.message = message;
  result.cost = cost;
  return result;
}

MissionPreflightResult Accept(const std::string& message, const MissionCostEstimate& cost) {
  MissionPreflightResult result;
  result.allowed = true;
  result.message = message;
  result.cost = cost;
  return result;
}

std::string FormatBatteryFailure(const MissionCostEstimate& estimate, const double minimum_voltage) {
  std::ostringstream out;
  out << "battery insufficient: projected " << estimate.projected_battery_voltage
      << "V below minimum " << minimum_voltage << "V";
  return out.str();
}

MissionCostEstimate EstimatePreflightCost(
    const MissionProfile& profile, const MissionPreflightConfig& config,
    const std::optional<double> distance_override_m) {
  if (distance_override_m.has_value()) {
    return EstimateDistanceCost(
        *distance_override_m, static_cast<int>(profile.waypoints.size()), config.nominal_speed_mps,
        config.battery_voltage, config.battery_drop_per_meter, config.minimum_battery_voltage,
        "planned_path");
  }
  return EstimateMissionCost(
      profile, config.nominal_speed_mps, config.battery_voltage, config.battery_drop_per_meter,
      config.minimum_battery_voltage);
}

std::optional<std::string> CheckSemanticZones(
    const MissionProfile& profile, const robot_navigation::ZoneCatalog& zone_catalog,
    const MissionPreflightConfig& config) {
  if (profile.frame_id != "map") {
    return "cannot run map semantic preflight for frame: " + profile.frame_id;
  }
  for (std::size_t i = 0; i < profile.waypoints.size(); ++i) {
    const auto& waypoint = profile.waypoints[i];
    const auto evaluation = zone_catalog.EvaluatePoint(
        config.map_name, waypoint.x, waypoint.y, config.default_speed_limit_mps);
    if (!evaluation.config_valid) {
      return evaluation.message;
    }
    if (!evaluation.allowed) {
      std::string zone_ids;
      for (const auto& zone : evaluation.matched_zones) {
        if (!zone_ids.empty()) {
          zone_ids += ",";
        }
        zone_ids += zone.zone_id;
      }
      return "waypoint " + std::to_string(i) + " is inside keepout zone(s): " + zone_ids;
    }
  }
  return std::nullopt;
}

}  // namespace

MissionPreflightResult ValidateMissionPreflight(
    const MissionProfile& profile, const robot_navigation::ZoneCatalog& zone_catalog,
    const MissionPreflightConfig& config, const std::optional<double> distance_override_m) {
  const auto cost = EstimatePreflightCost(profile, config, distance_override_m);
  if (!config.enabled) {
    return Accept("preflight disabled", cost);
  }

  std::string validation_message;
  if (!ValidateMissionProfile(profile, &validation_message)) {
    return Reject("invalid mission profile: " + validation_message, cost);
  }
  if (!cost.battery_sufficient) {
    return Reject(FormatBatteryFailure(cost, config.minimum_battery_voltage), cost);
  }

  try {
    if (auto zone_error = CheckSemanticZones(profile, zone_catalog, config)) {
      return Reject(*zone_error, cost);
    }
  } catch (const std::exception& error) {
    return Reject(std::string("failed to evaluate map semantic zones: ") + error.what(), cost);
  }

  std::ostringstream out;
  out << "preflight ok: " << cost.distance_m << "m, projected battery "
      << cost.projected_battery_voltage << "V";
  return Accept(out.str(), cost);
}

MissionPreflightResult ValidateStationTransportPreflight(
    const MissionProfile& profile, const StationCatalog& station_catalog,
    const std::string& pickup_station, const std::string& dropoff_station,
    const robot_navigation::ZoneCatalog& zone_catalog, const MissionPreflightConfig& config,
    const std::optional<double> path_distance_override_m) {
  const auto route_distance = EstimateRouteDistance(station_catalog, pickup_station, dropoff_station);
  if (config.enabled && config.require_station_route && !route_distance.has_value()) {
    return Reject("no enabled station route from " + pickup_station + " to " + dropoff_station);
  }
  return ValidateMissionPreflight(
      profile, zone_catalog, config,
      path_distance_override_m.has_value() ? path_distance_override_m : route_distance);
}

}  // namespace robot_tasks
