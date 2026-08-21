#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/station_catalog.hpp"

namespace robot_tasks {

struct FleetRobot {
  std::string id;
  bool enabled = true;
  std::string state = "available";
  double battery_voltage = 0.0;
  double max_payload_kg = 0.0;
  std::string current_station_id;
  std::vector<std::string> capabilities;
};

struct FleetCatalog {
  std::vector<FleetRobot> robots;
};

struct FleetRobotTaskScore {
  std::string robot_id;
  double approach_distance_m = 0.0;
  double task_distance_m = 0.0;
  double total_distance_m = 0.0;
  double battery_voltage = 0.0;
};

std::optional<FleetCatalog> LoadFleetCatalog(const std::filesystem::path& path);
const FleetRobot* FindFleetRobot(const FleetCatalog& catalog, const std::string& robot_id);
bool RobotHasCapability(const FleetRobot& robot, const std::string& capability);
const FleetRobot* SelectFleetRobot(
    const FleetCatalog& catalog, const std::string& required_capability,
    const std::vector<std::string>& unavailable_robot_ids);
const FleetRobot* SelectFleetRobotForStationTask(
    const FleetCatalog& fleet_catalog, const StationCatalog& station_catalog,
    const std::string& required_capability, const std::string& pickup_station,
    const std::string& dropoff_station, const std::vector<std::string>& unavailable_robot_ids,
    FleetRobotTaskScore* selected_score);
bool ValidateFleetCatalog(const FleetCatalog& catalog, std::string* message);

}  // namespace robot_tasks
