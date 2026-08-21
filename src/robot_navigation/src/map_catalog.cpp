#include "robot_navigation/map_catalog.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

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

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

}  // namespace

MapCatalog::MapCatalog(std::filesystem::path maps_directory)
    : maps_directory_(std::move(maps_directory)) {}

const std::filesystem::path& MapCatalog::MapsDirectory() const { return maps_directory_; }

std::vector<MapInfo> MapCatalog::Scan() const {
  std::vector<MapInfo> maps;
  if (!std::filesystem::exists(maps_directory_) ||
      !std::filesystem::is_directory(maps_directory_)) {
    return maps;
  }

  for (const auto& entry : std::filesystem::directory_iterator(maps_directory_)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
      continue;
    }
    if (auto info = LoadMapInfo(entry.path())) {
      maps.push_back(*info);
    }
  }
  std::sort(maps.begin(), maps.end(), [](const MapInfo& lhs, const MapInfo& rhs) {
    return lhs.name < rhs.name;
  });
  return maps;
}

std::optional<MapInfo> MapCatalog::FindByName(const std::string& name) const {
  for (const auto& info : Scan()) {
    if (info.name == name) {
      return info;
    }
  }
  return std::nullopt;
}

std::optional<MapInfo> LoadMapInfo(const std::filesystem::path& yaml_path) {
  std::ifstream input(yaml_path);
  if (!input) {
    return std::nullopt;
  }

  MapInfo info;
  info.name = yaml_path.stem().string();
  info.yaml_path = std::filesystem::absolute(yaml_path);
  std::string image;
  bool has_resolution = false;
  bool has_origin = false;
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (auto value = ValueAfterColon(line, "image")) {
      image = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "resolution")) {
      std::istringstream parser(*value);
      parser >> info.resolution;
      has_resolution = !parser.fail() && info.resolution > 0.0;
      continue;
    }
    if (auto value = ValueAfterColon(line, "origin")) {
      has_origin = !value->empty();
    }
  }

  if (image.empty()) {
    info.message = "missing image field";
    return std::nullopt;
  }
  info.image_path = std::filesystem::absolute(yaml_path.parent_path() / image);
  if (!std::filesystem::exists(info.image_path)) {
    info.message = "image file does not exist";
    return std::nullopt;
  }
  if (!has_resolution) {
    info.message = "missing or invalid resolution";
    return std::nullopt;
  }
  if (!has_origin) {
    info.message = "missing origin field";
    return std::nullopt;
  }
  info.valid = true;
  info.message = "ok";
  return info;
}

}  // namespace robot_navigation

