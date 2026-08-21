#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "robot_navigation/zone_catalog.hpp"
#include "robot_navigation/zone_filter_masks.hpp"

namespace {

void PrintUsage(const char* executable) {
  std::cerr << "usage: " << executable
            << " <map_yaml> <zones_yaml> <output_dir> [map_name]\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    auto map_info = robot_navigation::LoadOccupancyMapInfo(argv[1]);
    if (argc == 5) {
      map_info.map_name = argv[4];
    }
    const robot_navigation::ZoneCatalog zone_catalog(argv[2]);
    const auto zones = zone_catalog.ForMap(map_info.map_name, false);
    const auto grid = robot_navigation::BuildZoneFilterMaskGrid(map_info, zones);
    const auto summary = robot_navigation::WriteZoneFilterMaskAssets(map_info, grid, argv[3]);

    std::cout << "map_name=" << map_info.map_name << "\n";
    std::cout << "keepout_cells=" << summary.keepout_cells << "\n";
    std::cout << "speed_limit_cells=" << summary.speed_limit_cells << "\n";
    std::cout << "keepout_mask_yaml=" << summary.keepout_mask_yaml.string() << "\n";
    std::cout << "speed_mask_yaml=" << summary.speed_mask_yaml.string() << "\n";
    std::cout << "nav2_profile_yaml=" << summary.nav2_profile_yaml.string() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "export_zone_filter_masks failed: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
