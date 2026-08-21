#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_navigation {

struct MapZone {
  std::string zone_id;
  std::string map_name;
  std::string type;
  std::string frame_id = "map";
  bool enabled = true;
  double speed_limit_mps = 0.0;
  std::vector<double> polygon_x;
  std::vector<double> polygon_y;
  std::string rule;
};

struct ZoneEvaluation {
  bool config_valid = false;
  std::string message;
  bool allowed = true;
  double speed_limit_mps = 0.0;
  std::vector<MapZone> matched_zones;
};

class ZoneCatalog {
 public:
  explicit ZoneCatalog(std::filesystem::path zones_file);

  const std::filesystem::path& ZonesFile() const;
  std::vector<MapZone> Load(bool include_disabled = false) const;
  std::vector<MapZone> ForMap(const std::string& map_name, bool include_disabled = false) const;
  ZoneEvaluation EvaluatePoint(
      const std::string& map_name, double x, double y, double default_speed_limit_mps) const;

 private:
  std::filesystem::path zones_file_;
};

std::vector<MapZone> LoadMapZones(const std::filesystem::path& zones_file);
bool ContainsPoint(const MapZone& zone, double x, double y);
std::optional<std::string> ValidateZone(const MapZone& zone);

}  // namespace robot_navigation
