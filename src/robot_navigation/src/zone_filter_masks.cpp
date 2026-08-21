#include "robot_navigation/zone_filter_masks.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
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

std::string StripQuotes(std::string value) {
  value = Trim(value);
  if (value.size() >= 2U &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

std::string ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (!StartsWith(line, prefix)) {
    return "";
  }
  return StripQuotes(line.substr(prefix.size()));
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

std::vector<double> ParseVector3(const std::string& value, const std::string& field_name) {
  const auto open = value.find('[');
  const auto close = value.find(']', open == std::string::npos ? 0 : open);
  if (open == std::string::npos || close == std::string::npos) {
    throw std::runtime_error("invalid vector field " + field_name + ": " + value);
  }
  std::vector<double> parsed;
  std::istringstream input(value.substr(open + 1, close - open - 1));
  std::string token;
  while (std::getline(input, token, ',')) {
    parsed.push_back(ParseDouble(Trim(token), field_name));
  }
  if (parsed.size() != 3U) {
    throw std::runtime_error("expected 3 values for " + field_name);
  }
  return parsed;
}

std::string ReadPgmToken(std::istream& input) {
  std::string token;
  while (input >> token) {
    if (!token.empty() && token.front() == '#') {
      std::string rest_of_line;
      std::getline(input, rest_of_line);
      continue;
    }
    return token;
  }
  return "";
}

std::pair<int, int> ReadPgmSize(const std::filesystem::path& image_path) {
  std::ifstream input(image_path);
  if (!input) {
    throw std::runtime_error("failed to open map image: " + image_path.string());
  }
  const auto magic = ReadPgmToken(input);
  if (magic != "P2" && magic != "P5") {
    throw std::runtime_error("unsupported PGM magic in " + image_path.string() + ": " + magic);
  }
  const auto width_text = ReadPgmToken(input);
  const auto height_text = ReadPgmToken(input);
  if (width_text.empty() || height_text.empty()) {
    throw std::runtime_error("missing PGM size in " + image_path.string());
  }
  return {std::stoi(width_text), std::stoi(height_text)};
}

std::filesystem::path ResolveRelativeTo(
    const std::filesystem::path& base_file, const std::filesystem::path& maybe_relative) {
  if (maybe_relative.is_absolute()) {
    return maybe_relative;
  }
  return base_file.parent_path() / maybe_relative;
}

double CellCenterX(const OccupancyMapInfo& map_info, const int col) {
  return map_info.origin_x + (static_cast<double>(col) + 0.5) * map_info.resolution;
}

double CellCenterY(const OccupancyMapInfo& map_info, const int row) {
  return map_info.origin_y +
         (static_cast<double>(map_info.height - row) - 0.5) * map_info.resolution;
}

int SpeedLimitToMaskValue(const double speed_limit_mps) {
  if (speed_limit_mps <= 0.0) {
    return 0;
  }
  return std::clamp(static_cast<int>(std::lround(speed_limit_mps * 100.0)), 1, 100);
}

void WritePgm(const std::filesystem::path& path, const int width, const int height,
              const std::vector<int>& values) {
  if (static_cast<int>(values.size()) != width * height) {
    throw std::runtime_error("mask size does not match map dimensions for " + path.string());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << "P2\n";
  output << "# Generated from robot_navigation map_zones.yaml\n";
  output << width << " " << height << "\n";
  output << "100\n";
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      output << values[static_cast<std::size_t>(row * width + col)];
      if (col + 1 < width) {
        output << " ";
      }
    }
    output << "\n";
  }
}

void WriteMaskYaml(const std::filesystem::path& yaml_path, const std::filesystem::path& pgm_name,
                   const OccupancyMapInfo& map_info) {
  std::ofstream output(yaml_path);
  if (!output) {
    throw std::runtime_error("failed to write " + yaml_path.string());
  }
  output << "image: " << pgm_name.filename().string() << "\n";
  output << "mode: scale\n";
  output << "resolution: " << std::setprecision(12) << map_info.resolution << "\n";
  output << "origin: [" << map_info.origin_x << ", " << map_info.origin_y << ", "
         << map_info.origin_yaw << "]\n";
  output << "negate: 0\n";
  output << "occupied_thresh: 1.0\n";
  output << "free_thresh: 0.0\n";
}

void WriteNav2ProfileYaml(const std::filesystem::path& yaml_path,
                          const std::filesystem::path& keepout_mask_yaml,
                          const std::filesystem::path& speed_mask_yaml) {
  std::ofstream output(yaml_path);
  if (!output) {
    throw std::runtime_error("failed to write " + yaml_path.string());
  }
  output << "# Nav2 costmap filter profile generated from robot_navigation map zones.\n";
  output << "# Include these nodes/profile only after validating the mask servers in your launch.\n";
  output << "keepout_filter_mask_server:\n";
  output << "  ros__parameters:\n";
  output << "    yaml_filename: " << keepout_mask_yaml.filename().string() << "\n";
  output << "    topic_name: /keepout_filter_mask\n";
  output << "    frame_id: map\n";
  output << "speed_filter_mask_server:\n";
  output << "  ros__parameters:\n";
  output << "    yaml_filename: " << speed_mask_yaml.filename().string() << "\n";
  output << "    topic_name: /speed_filter_mask\n";
  output << "    frame_id: map\n";
  output << "keepout_costmap_filter_info_server:\n";
  output << "  ros__parameters:\n";
  output << "    filter_info_topic: /costmap_filter_info/keepout\n";
  output << "    type: 0\n";
  output << "    mask_topic: /keepout_filter_mask\n";
  output << "    base: 0.0\n";
  output << "    multiplier: 1.0\n";
  output << "speed_costmap_filter_info_server:\n";
  output << "  ros__parameters:\n";
  output << "    filter_info_topic: /costmap_filter_info/speed\n";
  output << "    type: 2\n";
  output << "    mask_topic: /speed_filter_mask\n";
  output << "    base: 0.0\n";
  output << "    multiplier: 0.01\n";
  output << "global_costmap:\n";
  output << "  global_costmap:\n";
  output << "    ros__parameters:\n";
  output << "      filters: [\"keepout_filter\", \"speed_filter\"]\n";
  output << "      keepout_filter:\n";
  output << "        plugin: nav2_costmap_2d::KeepoutFilter\n";
  output << "        enabled: true\n";
  output << "        filter_info_topic: /costmap_filter_info/keepout\n";
  output << "      speed_filter:\n";
  output << "        plugin: nav2_costmap_2d::SpeedFilter\n";
  output << "        enabled: true\n";
  output << "        filter_info_topic: /costmap_filter_info/speed\n";
  output << "        speed_limit_topic: /speed_limit\n";
  output << "controller_server:\n";
  output << "  ros__parameters:\n";
  output << "    speed_limit_topic: /speed_limit\n";
}

}  // namespace

OccupancyMapInfo LoadOccupancyMapInfo(const std::filesystem::path& map_yaml) {
  std::ifstream input(map_yaml);
  if (!input) {
    throw std::runtime_error("failed to open map yaml: " + map_yaml.string());
  }

  OccupancyMapInfo map_info;
  map_info.yaml_path = map_yaml;
  map_info.map_name = map_yaml.stem().string();
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (StartsWith(line, "image:")) {
      map_info.image_path = ResolveRelativeTo(map_yaml, ValueAfterColon(line, "image"));
    } else if (StartsWith(line, "resolution:")) {
      map_info.resolution = ParseDouble(ValueAfterColon(line, "resolution"), "resolution");
    } else if (StartsWith(line, "origin:")) {
      const auto origin = ParseVector3(ValueAfterColon(line, "origin"), "origin");
      map_info.origin_x = origin[0];
      map_info.origin_y = origin[1];
      map_info.origin_yaw = origin[2];
    }
  }

  if (map_info.image_path.empty()) {
    throw std::runtime_error("map yaml is missing image: " + map_yaml.string());
  }
  if (map_info.resolution <= 0.0) {
    throw std::runtime_error("map yaml requires positive resolution: " + map_yaml.string());
  }
  const auto [width, height] = ReadPgmSize(map_info.image_path);
  map_info.width = width;
  map_info.height = height;
  return map_info;
}

ZoneFilterMaskGrid BuildZoneFilterMaskGrid(
    const OccupancyMapInfo& map_info, const std::vector<MapZone>& zones) {
  if (map_info.width <= 0 || map_info.height <= 0 || map_info.resolution <= 0.0) {
    throw std::runtime_error("invalid map metadata for zone filter mask generation");
  }

  ZoneFilterMaskGrid grid;
  grid.width = map_info.width;
  grid.height = map_info.height;
  grid.resolution = map_info.resolution;
  grid.origin_x = map_info.origin_x;
  grid.origin_y = map_info.origin_y;
  grid.origin_yaw = map_info.origin_yaw;
  grid.keepout_values.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  grid.speed_values.assign(static_cast<std::size_t>(grid.width * grid.height), 0);

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const double x = CellCenterX(map_info, col);
      const double y = CellCenterY(map_info, row);
      bool keepout = false;
      double speed_limit_mps = 0.0;
      for (const auto& zone : zones) {
        if (!zone.enabled || zone.map_name != map_info.map_name || !ContainsPoint(zone, x, y)) {
          continue;
        }
        const auto type = ToLower(zone.type);
        if (type == "keepout") {
          keepout = true;
        } else if (type == "speed_limit" && zone.speed_limit_mps > 0.0) {
          speed_limit_mps = speed_limit_mps <= 0.0
                                ? zone.speed_limit_mps
                                : std::min(speed_limit_mps, zone.speed_limit_mps);
        }
      }
      const auto index = static_cast<std::size_t>(row * grid.width + col);
      if (keepout) {
        grid.keepout_values[index] = 100;
        ++grid.keepout_cells;
      }
      const int speed_value = SpeedLimitToMaskValue(speed_limit_mps);
      if (speed_value > 0) {
        grid.speed_values[index] = speed_value;
        ++grid.speed_limit_cells;
      }
    }
  }
  return grid;
}

ZoneFilterExportSummary WriteZoneFilterMaskAssets(
    const OccupancyMapInfo& map_info, const ZoneFilterMaskGrid& grid,
    const std::filesystem::path& output_dir) {
  std::filesystem::create_directories(output_dir);
  const auto keepout_pgm = output_dir / (map_info.map_name + "_keepout_mask.pgm");
  const auto keepout_yaml = output_dir / (map_info.map_name + "_keepout_mask.yaml");
  const auto speed_pgm = output_dir / (map_info.map_name + "_speed_mask.pgm");
  const auto speed_yaml = output_dir / (map_info.map_name + "_speed_mask.yaml");
  const auto nav2_profile = output_dir / (map_info.map_name + "_nav2_costmap_filters.yaml");

  WritePgm(keepout_pgm, grid.width, grid.height, grid.keepout_values);
  WritePgm(speed_pgm, grid.width, grid.height, grid.speed_values);
  WriteMaskYaml(keepout_yaml, keepout_pgm, map_info);
  WriteMaskYaml(speed_yaml, speed_pgm, map_info);
  WriteNav2ProfileYaml(nav2_profile, keepout_yaml, speed_yaml);

  ZoneFilterExportSummary summary;
  summary.keepout_mask_yaml = keepout_yaml;
  summary.speed_mask_yaml = speed_yaml;
  summary.nav2_profile_yaml = nav2_profile;
  summary.keepout_cells = grid.keepout_cells;
  summary.speed_limit_cells = grid.speed_limit_cells;
  return summary;
}

}  // namespace robot_navigation
