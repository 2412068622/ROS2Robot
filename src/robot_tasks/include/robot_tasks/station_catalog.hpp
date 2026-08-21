#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

struct Station {
  std::string id;
  std::string frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct StationEdge {
  std::string from;
  std::string to;
  double distance_m = 0.0;
  bool bidirectional = true;
  bool enabled = true;
};

struct StationCatalog {
  std::vector<Station> stations;
  std::vector<StationEdge> edges;
};

struct StationListProjection {
  std::vector<std::string> station_ids;
  std::vector<std::string> frame_ids;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::string message;
};

struct StationRouteListProjection {
  std::vector<std::string> from_station;
  std::vector<std::string> to_station;
  std::vector<double> distance_m;
  std::vector<bool> bidirectional;
  std::vector<bool> enabled;
  std::string message;
};

std::optional<StationCatalog> LoadStationCatalog(const std::filesystem::path& path);
const Station* FindStation(const StationCatalog& catalog, const std::string& id);
std::optional<double> EstimateRouteDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
std::optional<std::vector<std::string>> FindRoutePath(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message);
StationListProjection BuildStationListProjection(const StationCatalog& catalog);
StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog);

}  // namespace robot_tasks
