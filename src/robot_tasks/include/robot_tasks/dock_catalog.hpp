#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/docking_workflow.hpp"

namespace robot_tasks {

struct Dock {
  std::string id;
  std::string station_id;
  std::string approach_station_id;
  std::string charger_resource_id;
  bool enabled = true;
};

struct DockCatalog {
  std::vector<Dock> docks;
};

struct DockStateProjection {
  std::string dock_id;
  std::string station_id;
  std::string approach_station_id;
  std::string charger_resource_id;
  bool enabled = false;
  bool available = false;
  std::string state;
  std::string mission_id;
};

struct DockListProjection {
  std::vector<std::string> dock_ids;
  std::vector<std::string> station_ids;
  std::vector<std::string> approach_station_ids;
  std::vector<std::string> charger_resource_ids;
  std::vector<bool> enabled;
  std::vector<bool> available;
  std::vector<std::string> states;
  std::vector<std::string> mission_ids;
  std::string message;
};

std::optional<DockCatalog> LoadDockCatalog(const std::filesystem::path& path);
const Dock* FindDock(const DockCatalog& catalog, const std::string& dock_id);
const Dock* FindFirstEnabledDock(const DockCatalog& catalog);
bool ValidateDockCatalog(const DockCatalog& catalog, std::string* message);
DockStateProjection BuildDockStateProjection(
    const Dock& dock, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations);
DockListProjection BuildDockListProjection(
    const DockCatalog& catalog, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations, bool include_disabled);

}  // namespace robot_tasks
