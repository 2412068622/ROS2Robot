#include "robot_tasks/mission_profile.hpp"

#include <fstream>
#include <sstream>
#include <string>

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

std::optional<double> ParseDouble(const std::string& value) {
  std::istringstream parser(value);
  double result = 0.0;
  parser >> result;
  if (parser.fail()) {
    return std::nullopt;
  }
  return result;
}

}  // namespace

std::optional<MissionProfile> LoadMissionProfile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  MissionProfile profile;
  MissionWaypoint current_waypoint;
  bool in_waypoint = false;
  bool has_x = false;
  bool has_y = false;
  bool has_yaw = false;

  auto flush_waypoint = [&]() {
    if (in_waypoint && has_x && has_y) {
      profile.waypoints.push_back(current_waypoint);
    }
    current_waypoint = MissionWaypoint{};
    in_waypoint = false;
    has_x = false;
    has_y = false;
    has_yaw = false;
  };

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    if (auto value = ValueAfterColon(line, "mission_id")) {
      profile.mission_id = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "frame_id")) {
      profile.frame_id = value->empty() ? "map" : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "loop")) {
      profile.loop = ParseBool(*value);
      continue;
    }

    if (line.rfind("- ", 0) == 0) {
      flush_waypoint();
      in_waypoint = true;
      line = Trim(line.substr(2));
    }

    if (auto value = ValueAfterColon(line, "x")) {
      if (auto parsed = ParseDouble(*value)) {
        current_waypoint.x = *parsed;
        has_x = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "y")) {
      if (auto parsed = ParseDouble(*value)) {
        current_waypoint.y = *parsed;
        has_y = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "yaw")) {
      if (auto parsed = ParseDouble(*value)) {
        current_waypoint.yaw = *parsed;
        has_yaw = true;
      }
      continue;
    }
  }
  (void)has_yaw;
  flush_waypoint();

  if (profile.mission_id.empty()) {
    profile.mission_id = path.stem().string();
  }
  std::string message;
  if (!ValidateMissionProfile(profile, &message)) {
    return std::nullopt;
  }
  return profile;
}

bool ValidateMissionProfile(const MissionProfile& profile, std::string* message) {
  if (profile.mission_id.empty()) {
    if (message != nullptr) {
      *message = "mission_id is empty";
    }
    return false;
  }
  if (profile.frame_id.empty()) {
    if (message != nullptr) {
      *message = "frame_id is empty";
    }
    return false;
  }
  if (profile.waypoints.empty()) {
    if (message != nullptr) {
      *message = "mission has no waypoints";
    }
    return false;
  }
  if (message != nullptr) {
    *message = "ok";
  }
  return true;
}

}  // namespace robot_tasks

