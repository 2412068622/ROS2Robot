#include "robot_navigation/zone_catalog.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace robot_navigation {
namespace {

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (!StartsWith(line, prefix)) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

bool ParseBool(const std::string& value) {
  const auto lowered = ToLower(Trim(value));
  return lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on";
}

double ParseDouble(const std::string& value, const std::string& field_name) {
  std::istringstream parser(value);
  double result = 0.0;
  parser >> result;
  if (parser.fail()) {
    throw std::runtime_error("invalid numeric field " + field_name + ": " + value);
  }
  return result;
}

std::optional<std::pair<double, double>> ParsePoint(const std::string& line) {
  const auto open = line.find('[');
  const auto comma = line.find(',', open == std::string::npos ? 0 : open);
  const auto close = line.find(']', comma == std::string::npos ? 0 : comma);
  if (open == std::string::npos || comma == std::string::npos || close == std::string::npos) {
    return std::nullopt;
  }
  const auto x_text = Trim(line.substr(open + 1, comma - open - 1));
  const auto y_text = Trim(line.substr(comma + 1, close - comma - 1));
  return std::make_pair(ParseDouble(x_text, "polygon.x"), ParseDouble(y_text, "polygon.y"));
}

void SortZones(std::vector<MapZone>& zones) {
  std::sort(zones.begin(), zones.end(), [](const MapZone& lhs, const MapZone& rhs) {
    if (lhs.map_name != rhs.map_name) {
      return lhs.map_name < rhs.map_name;
    }
    return lhs.zone_id < rhs.zone_id;
  });
}

}  // namespace

ZoneCatalog::ZoneCatalog(std::filesystem::path zones_file) : zones_file_(std::move(zones_file)) {}

const std::filesystem::path& ZoneCatalog::ZonesFile() const { return zones_file_; }

std::vector<MapZone> ZoneCatalog::Load(const bool include_disabled) const {
  auto zones = LoadMapZones(zones_file_);
  if (include_disabled) {
    return zones;
  }
  zones.erase(
      std::remove_if(zones.begin(), zones.end(), [](const MapZone& zone) { return !zone.enabled; }),
      zones.end());
  return zones;
}

std::vector<MapZone> ZoneCatalog::ForMap(
    const std::string& map_name, const bool include_disabled) const {
  auto zones = Load(include_disabled);
  zones.erase(
      std::remove_if(
          zones.begin(), zones.end(),
          [&map_name](const MapZone& zone) { return zone.map_name != map_name; }),
      zones.end());
  return zones;
}

ZoneEvaluation ZoneCatalog::EvaluatePoint(
    const std::string& map_name, const double x, const double y,
    const double default_speed_limit_mps) const {
  ZoneEvaluation result;
  result.config_valid = true;
  result.message = "ok";
  result.allowed = true;
  result.speed_limit_mps = default_speed_limit_mps;

  for (const auto& zone : ForMap(map_name, false)) {
    if (!ContainsPoint(zone, x, y)) {
      continue;
    }
    result.matched_zones.push_back(zone);
    const auto type = ToLower(zone.type);
    if (type == "keepout") {
      result.allowed = false;
    } else if (type == "speed_limit" && zone.speed_limit_mps > 0.0) {
      result.speed_limit_mps = std::min(result.speed_limit_mps, zone.speed_limit_mps);
    }
  }

  if (result.matched_zones.empty()) {
    result.message = "no matching zones";
  } else if (!result.allowed) {
    result.message = "point is inside keepout zone";
  } else {
    result.message = "point matched semantic zone";
  }
  return result;
}

std::vector<MapZone> LoadMapZones(const std::filesystem::path& zones_file) {
  std::ifstream input(zones_file);
  if (!input) {
    return {};
  }

  std::vector<MapZone> zones;
  std::optional<MapZone> current;
  bool reading_polygon = false;
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "zones:") {
      continue;
    }
    if (StartsWith(line, "- zone_id:")) {
      if (current.has_value()) {
        if (auto error = ValidateZone(*current)) {
          throw std::runtime_error(*error);
        }
        zones.push_back(*current);
      }
      current = MapZone{};
      current->zone_id = Trim(line.substr(std::string("- zone_id:").size()));
      reading_polygon = false;
      continue;
    }
    if (!current.has_value()) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "map_name")) {
      current->map_name = *value;
      reading_polygon = false;
      continue;
    }
    if (auto value = ValueAfterColon(line, "type")) {
      current->type = *value;
      reading_polygon = false;
      continue;
    }
    if (auto value = ValueAfterColon(line, "frame_id")) {
      current->frame_id = *value;
      reading_polygon = false;
      continue;
    }
    if (auto value = ValueAfterColon(line, "enabled")) {
      current->enabled = ParseBool(*value);
      reading_polygon = false;
      continue;
    }
    if (auto value = ValueAfterColon(line, "speed_limit_mps")) {
      current->speed_limit_mps = ParseDouble(*value, "speed_limit_mps");
      reading_polygon = false;
      continue;
    }
    if (auto value = ValueAfterColon(line, "rule")) {
      current->rule = *value;
      reading_polygon = false;
      continue;
    }
    if (StartsWith(line, "polygon:")) {
      reading_polygon = true;
      continue;
    }
    if (reading_polygon && StartsWith(line, "- [")) {
      const auto point = ParsePoint(line);
      if (!point.has_value()) {
        throw std::runtime_error("invalid polygon point in " + current->zone_id);
      }
      current->polygon_x.push_back(point->first);
      current->polygon_y.push_back(point->second);
    }
  }
  if (current.has_value()) {
    if (auto error = ValidateZone(*current)) {
      throw std::runtime_error(*error);
    }
    zones.push_back(*current);
  }
  SortZones(zones);
  return zones;
}

bool ContainsPoint(const MapZone& zone, const double x, const double y) {
  const auto count = zone.polygon_x.size();
  if (count < 3 || zone.polygon_y.size() != count) {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
    const double xi = zone.polygon_x[i];
    const double yi = zone.polygon_y[i];
    const double xj = zone.polygon_x[j];
    const double yj = zone.polygon_y[j];
    const bool crosses = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

std::optional<std::string> ValidateZone(const MapZone& zone) {
  if (zone.zone_id.empty()) {
    return "zone_id is required";
  }
  if (zone.map_name.empty()) {
    return "map_name is required for zone " + zone.zone_id;
  }
  if (zone.type.empty()) {
    return "type is required for zone " + zone.zone_id;
  }
  if (zone.frame_id.empty()) {
    return "frame_id is required for zone " + zone.zone_id;
  }
  if (zone.polygon_x.size() != zone.polygon_y.size() || zone.polygon_x.size() < 3U) {
    return "polygon requires at least 3 points for zone " + zone.zone_id;
  }
  const auto type = ToLower(zone.type);
  if (type != "keepout" && type != "speed_limit" && type != "preferred_lane") {
    return "unsupported zone type for zone " + zone.zone_id + ": " + zone.type;
  }
  if (type == "speed_limit" && zone.speed_limit_mps <= 0.0) {
    return "speed_limit zone requires positive speed_limit_mps for zone " + zone.zone_id;
  }
  return std::nullopt;
}

}  // namespace robot_navigation
