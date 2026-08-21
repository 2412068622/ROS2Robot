#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_navigation {

struct MapInfo {
  std::string name;
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  double resolution = 0.0;
  bool valid = false;
  std::string message;
};

class MapCatalog {
 public:
  explicit MapCatalog(std::filesystem::path maps_directory);

  const std::filesystem::path& MapsDirectory() const;
  std::vector<MapInfo> Scan() const;
  std::optional<MapInfo> FindByName(const std::string& name) const;

 private:
  std::filesystem::path maps_directory_;
};

std::optional<MapInfo> LoadMapInfo(const std::filesystem::path& yaml_path);

}  // namespace robot_navigation

