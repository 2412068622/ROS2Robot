#pragma once

#include <optional>
#include <string>

#include "robot_tasks/mission_estimator.hpp"
#include "robot_tasks/mission_profile.hpp"
#include "robot_tasks/station_catalog.hpp"

namespace robot_navigation {
class ZoneCatalog;
}

namespace robot_tasks {

struct MissionPreflightConfig {
  bool enabled = true;
  std::string map_name = "indoor_room";
  double default_speed_limit_mps = 0.6;
  double nominal_speed_mps = 0.35;
  double battery_voltage = 24.0;
  double battery_drop_per_meter = 0.015;
  double minimum_battery_voltage = 22.0;
  bool require_station_route = true;
};

struct MissionPreflightResult {
  bool allowed = true;
  std::string message = "preflight ok";
  MissionCostEstimate cost;
};

MissionPreflightResult ValidateMissionPreflight(
    const MissionProfile& profile, const robot_navigation::ZoneCatalog& zone_catalog,
    const MissionPreflightConfig& config,
    std::optional<double> distance_override_m = std::nullopt);

MissionPreflightResult ValidateStationTransportPreflight(
    const MissionProfile& profile, const StationCatalog& station_catalog,
    const std::string& pickup_station, const std::string& dropoff_station,
    const robot_navigation::ZoneCatalog& zone_catalog, const MissionPreflightConfig& config,
    std::optional<double> path_distance_override_m = std::nullopt);

}  // namespace robot_tasks
