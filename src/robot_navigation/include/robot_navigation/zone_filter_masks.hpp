#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "robot_navigation/zone_catalog.hpp"

namespace robot_navigation {

struct OccupancyMapInfo {
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  std::string map_name;
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
};

struct ZoneFilterMaskGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<int> keepout_values;
  std::vector<int> speed_values;
  int keepout_cells = 0;
  int speed_limit_cells = 0;
};

struct ZoneFilterExportSummary {
  std::filesystem::path keepout_mask_yaml;
  std::filesystem::path speed_mask_yaml;
  std::filesystem::path nav2_profile_yaml;
  int keepout_cells = 0;
  int speed_limit_cells = 0;
};

OccupancyMapInfo LoadOccupancyMapInfo(const std::filesystem::path& map_yaml);
ZoneFilterMaskGrid BuildZoneFilterMaskGrid(
    const OccupancyMapInfo& map_info, const std::vector<MapZone>& zones);
ZoneFilterExportSummary WriteZoneFilterMaskAssets(
    const OccupancyMapInfo& map_info, const ZoneFilterMaskGrid& grid,
    const std::filesystem::path& output_dir);

}  // namespace robot_navigation
