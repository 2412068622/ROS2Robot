#include "robot_tasks/dock_catalog.hpp"

#include <fstream>
#include <unordered_set>

namespace robot_tasks {
namespace {

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

}  // namespace

std::optional<DockCatalog> LoadDockCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  DockCatalog catalog;
  Dock current;
  bool in_dock = false;
  bool has_id = false;
  bool has_station = false;
  bool has_charger = false;

  auto flush_dock = [&]() {
    if (in_dock && has_id && has_station && has_charger) {
      if (current.approach_station_id.empty()) {
        current.approach_station_id = current.station_id;
      }
      catalog.docks.push_back(current);
    }
    current = Dock{};
    in_dock = false;
    has_id = false;
    has_station = false;
    has_charger = false;
  };

  std::string section;
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "docks:") {
      flush_dock();
      section = "docks";
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      if (section != "docks") {
        continue;
      }
      flush_dock();
      in_dock = true;
      line = Trim(line.substr(2));
    }
    if (!in_dock) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "id")) {
      current.id = *value;
      has_id = !current.id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "station_id")) {
      current.station_id = *value;
      has_station = !current.station_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "approach_station_id")) {
      current.approach_station_id = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "charger_resource_id")) {
      current.charger_resource_id = *value;
      has_charger = !current.charger_resource_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "enabled")) {
      current.enabled = ParseBool(*value);
      continue;
    }
  }
  flush_dock();

  std::string message;
  if (!ValidateDockCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const Dock* FindDock(const DockCatalog& catalog, const std::string& dock_id) {
  for (const auto& dock : catalog.docks) {
    if (dock.id == dock_id) {
      return &dock;
    }
  }
  return nullptr;
}

const Dock* FindFirstEnabledDock(const DockCatalog& catalog) {
  for (const auto& dock : catalog.docks) {
    if (dock.enabled) {
      return &dock;
    }
  }
  return nullptr;
}

bool ValidateDockCatalog(const DockCatalog& catalog, std::string* message) {
  std::unordered_set<std::string> ids;
  for (const auto& dock : catalog.docks) {
    if (dock.id.empty()) {
      if (message != nullptr) {
        *message = "dock id is empty";
      }
      return false;
    }
    if (dock.station_id.empty()) {
      if (message != nullptr) {
        *message = "dock station_id is empty: " + dock.id;
      }
      return false;
    }
    if (dock.charger_resource_id.empty()) {
      if (message != nullptr) {
        *message = "dock charger_resource_id is empty: " + dock.id;
      }
      return false;
    }
    if (!ids.insert(dock.id).second) {
      if (message != nullptr) {
        *message = "duplicate dock id: " + dock.id;
      }
      return false;
    }
  }
  if (message != nullptr) {
    *message = "dock catalog ok";
  }
  return true;
}

DockStateProjection BuildDockStateProjection(
    const Dock& dock, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations) {
  DockStateProjection projection;
  projection.dock_id = dock.id;
  projection.station_id = dock.station_id;
  projection.approach_station_id = dock.approach_station_id;
  projection.charger_resource_id = dock.charger_resource_id;
  projection.enabled = dock.enabled;
  projection.available = dock.enabled && reservations.find(dock.charger_resource_id) == reservations.end();
  projection.state = dock.enabled ? "AVAILABLE" : "DISABLED";

  const auto reservation = reservations.find(dock.charger_resource_id);
  if (reservation != reservations.end()) {
    projection.state = reservation->second.status;
    projection.mission_id = reservation->second.mission_id;
  }
  const auto runtime = dock_states.find(dock.id);
  if (runtime != dock_states.end()) {
    projection.state = runtime->second.state;
    projection.mission_id = runtime->second.mission_id;
  }
  return projection;
}

DockListProjection BuildDockListProjection(
    const DockCatalog& catalog, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations, const bool include_disabled) {
  DockListProjection projection;
  projection.dock_ids.reserve(catalog.docks.size());
  projection.station_ids.reserve(catalog.docks.size());
  projection.approach_station_ids.reserve(catalog.docks.size());
  projection.charger_resource_ids.reserve(catalog.docks.size());
  projection.enabled.reserve(catalog.docks.size());
  projection.available.reserve(catalog.docks.size());
  projection.states.reserve(catalog.docks.size());
  projection.mission_ids.reserve(catalog.docks.size());
  for (const auto& dock : catalog.docks) {
    if (!include_disabled && !dock.enabled) {
      continue;
    }
    const auto state = BuildDockStateProjection(dock, dock_states, reservations);
    projection.dock_ids.push_back(state.dock_id);
    projection.station_ids.push_back(state.station_id);
    projection.approach_station_ids.push_back(state.approach_station_id);
    projection.charger_resource_ids.push_back(state.charger_resource_id);
    projection.enabled.push_back(state.enabled);
    projection.available.push_back(state.available);
    projection.states.push_back(state.state);
    projection.mission_ids.push_back(state.mission_id);
  }
  projection.message = "loaded " + std::to_string(projection.dock_ids.size()) + " dock(s)";
  return projection;
}

}  // namespace robot_tasks
