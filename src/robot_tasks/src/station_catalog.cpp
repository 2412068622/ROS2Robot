#include "robot_tasks/station_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

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

std::optional<double> ParseDouble(const std::string& value) {
  std::istringstream parser(value);
  double result = 0.0;
  parser >> result;
  if (parser.fail() || !std::isfinite(result)) {
    return std::nullopt;
  }
  return result;
}

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

}  // namespace

std::optional<StationCatalog> LoadStationCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  StationCatalog catalog;
  Station current_station;
  StationEdge current_edge;
  std::string default_frame_id = "map";
  std::string section;
  bool in_station = false;
  bool in_edge = false;
  bool has_id = false;
  bool has_x = false;
  bool has_y = false;
  bool has_from = false;
  bool has_to = false;
  bool has_distance = false;

  auto flush_station = [&]() {
    if (in_station && has_id && has_x && has_y) {
      if (current_station.frame_id.empty()) {
        current_station.frame_id = default_frame_id;
      }
      catalog.stations.push_back(current_station);
    }
    current_station = Station{};
    current_station.frame_id = default_frame_id;
    in_station = false;
    has_id = false;
    has_x = false;
    has_y = false;
  };

  auto flush_edge = [&]() {
    if (in_edge && has_from && has_to && has_distance) {
      catalog.edges.push_back(current_edge);
    }
    current_edge = StationEdge{};
    in_edge = false;
    has_from = false;
    has_to = false;
    has_distance = false;
  };

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    if (line == "stations:" || line == "edges:" || line == "routes:") {
      flush_station();
      flush_edge();
      section = line.substr(0, line.size() - 1);
      continue;
    }

    if (!in_station && !in_edge) {
      if (auto value = ValueAfterColon(line, "frame_id")) {
        default_frame_id = value->empty() ? "map" : *value;
        current_station.frame_id = default_frame_id;
        continue;
      }
    }

    if (line.rfind("- ", 0) == 0) {
      if (section == "edges" || section == "routes") {
        flush_edge();
        in_edge = true;
      } else {
        flush_station();
        in_station = true;
        current_station.frame_id = default_frame_id;
      }
      line = Trim(line.substr(2));
    }

    if (in_edge) {
      if (auto value = ValueAfterColon(line, "from")) {
        current_edge.from = *value;
        has_from = !current_edge.from.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "to")) {
        current_edge.to = *value;
        has_to = !current_edge.to.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "distance_m")) {
        if (auto parsed = ParseDouble(*value)) {
          current_edge.distance_m = *parsed;
          has_distance = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "distance")) {
        if (auto parsed = ParseDouble(*value)) {
          current_edge.distance_m = *parsed;
          has_distance = true;
        }
        continue;
      }
      if (auto value = ValueAfterColon(line, "bidirectional")) {
        current_edge.bidirectional = ParseBool(*value);
        continue;
      }
      if (auto value = ValueAfterColon(line, "enabled")) {
        current_edge.enabled = ParseBool(*value);
        continue;
      }
      continue;
    }

    if (auto value = ValueAfterColon(line, "id")) {
      current_station.id = *value;
      has_id = !current_station.id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "frame_id")) {
      current_station.frame_id = value->empty() ? default_frame_id : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "x")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.x = *parsed;
        has_x = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "y")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.y = *parsed;
        has_y = true;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "yaw")) {
      if (auto parsed = ParseDouble(*value)) {
        current_station.yaw = *parsed;
      }
      continue;
    }
  }
  flush_station();
  flush_edge();

  std::string message;
  if (!ValidateStationCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const Station* FindStation(const StationCatalog& catalog, const std::string& id) {
  for (const auto& station : catalog.stations) {
    if (station.id == id) {
      return &station;
    }
  }
  return nullptr;
}

std::optional<double> EstimateRouteDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to) {
  const auto path = FindRoutePath(catalog, from, to);
  if (!path.has_value()) {
    return std::nullopt;
  }
  double distance_m = 0.0;
  for (std::size_t index = 1; index < path->size(); ++index) {
    for (const auto& edge : catalog.edges) {
      const bool forward = edge.from == (*path)[index - 1] && edge.to == (*path)[index];
      const bool reverse =
          edge.bidirectional && edge.to == (*path)[index - 1] && edge.from == (*path)[index];
      if (edge.enabled && (forward || reverse)) {
        distance_m += edge.distance_m;
        break;
      }
    }
  }
  return distance_m;
}

std::optional<std::vector<std::string>> FindRoutePath(
    const StationCatalog& catalog, const std::string& from, const std::string& to) {
  if (from == to) {
    return std::vector<std::string>{from};
  }

  std::vector<std::string> ids;
  ids.reserve(catalog.stations.size());
  for (const auto& station : catalog.stations) {
    ids.push_back(station.id);
  }
  const auto index_of = [&](const std::string& id) -> std::optional<std::size_t> {
    const auto it = std::find(ids.begin(), ids.end(), id);
    if (it == ids.end()) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(ids.begin(), it));
  };

  const auto start = index_of(from);
  const auto goal = index_of(to);
  if (!start.has_value() || !goal.has_value()) {
    return std::nullopt;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distance(ids.size(), infinity);
  std::vector<std::optional<std::size_t>> previous(ids.size(), std::nullopt);
  std::vector<bool> visited(ids.size(), false);
  distance[*start] = 0.0;

  for (std::size_t iteration = 0; iteration < ids.size(); ++iteration) {
    std::optional<std::size_t> current;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (!visited[i] && (!current.has_value() || distance[i] < distance[*current])) {
        current = i;
      }
    }
    if (!current.has_value() || !std::isfinite(distance[*current])) {
      break;
    }
    if (*current == *goal) {
      break;
    }
    visited[*current] = true;

    for (const auto& edge : catalog.edges) {
      if (!edge.enabled) {
        continue;
      }
      const auto edge_from = index_of(edge.from);
      const auto edge_to = index_of(edge.to);
      if (!edge_from.has_value() || !edge_to.has_value()) {
        continue;
      }
      auto relax = [&](const std::size_t source, const std::size_t target) {
        if (source == *current && distance[source] + edge.distance_m < distance[target]) {
          distance[target] = distance[source] + edge.distance_m;
          previous[target] = source;
        }
      };
      relax(*edge_from, *edge_to);
      if (edge.bidirectional) {
        relax(*edge_to, *edge_from);
      }
    }
  }
  if (!std::isfinite(distance[*goal])) {
    return std::nullopt;
  }
  std::vector<std::string> path;
  for (auto current = *goal;;) {
    path.push_back(ids[current]);
    if (current == *start) {
      break;
    }
    if (!previous[current].has_value()) {
      return std::nullopt;
    }
    current = *previous[current];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message) {
  if (catalog.stations.empty()) {
    if (message != nullptr) {
      *message = "station catalog is empty";
    }
    return false;
  }

  std::unordered_set<std::string> ids;
  for (const auto& station : catalog.stations) {
    if (station.id.empty()) {
      if (message != nullptr) {
        *message = "station id is empty";
      }
      return false;
    }
    if (station.frame_id.empty()) {
      if (message != nullptr) {
        *message = "station frame_id is empty: " + station.id;
      }
      return false;
    }
    if (!std::isfinite(station.x) || !std::isfinite(station.y) || !std::isfinite(station.yaw)) {
      if (message != nullptr) {
        *message = "station pose is not finite: " + station.id;
      }
      return false;
    }
    if (!ids.insert(station.id).second) {
      if (message != nullptr) {
        *message = "duplicate station id: " + station.id;
      }
      return false;
    }
  }

  for (const auto& edge : catalog.edges) {
    if (edge.from.empty() || edge.to.empty()) {
      if (message != nullptr) {
        *message = "station edge endpoint is empty";
      }
      return false;
    }
    if (ids.find(edge.from) == ids.end()) {
      if (message != nullptr) {
        *message = "station edge has unknown from endpoint: " + edge.from;
      }
      return false;
    }
    if (ids.find(edge.to) == ids.end()) {
      if (message != nullptr) {
        *message = "station edge has unknown to endpoint: " + edge.to;
      }
      return false;
    }
    if (!std::isfinite(edge.distance_m) || edge.distance_m < 0.0) {
      if (message != nullptr) {
        *message = "station edge distance is invalid: " + edge.from + " -> " + edge.to;
      }
      return false;
    }
  }

  if (message != nullptr) {
    *message = "ok";
  }
  return true;
}

StationListProjection BuildStationListProjection(const StationCatalog& catalog) {
  StationListProjection projection;
  projection.station_ids.reserve(catalog.stations.size());
  projection.frame_ids.reserve(catalog.stations.size());
  projection.x.reserve(catalog.stations.size());
  projection.y.reserve(catalog.stations.size());
  projection.yaw.reserve(catalog.stations.size());
  for (const auto& station : catalog.stations) {
    projection.station_ids.push_back(station.id);
    projection.frame_ids.push_back(station.frame_id);
    projection.x.push_back(station.x);
    projection.y.push_back(station.y);
    projection.yaw.push_back(station.yaw);
  }
  projection.message = "loaded " + std::to_string(catalog.stations.size()) + " station(s)";
  return projection;
}

StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog) {
  StationRouteListProjection projection;
  projection.from_station.reserve(catalog.edges.size());
  projection.to_station.reserve(catalog.edges.size());
  projection.distance_m.reserve(catalog.edges.size());
  projection.bidirectional.reserve(catalog.edges.size());
  projection.enabled.reserve(catalog.edges.size());
  for (const auto& edge : catalog.edges) {
    projection.from_station.push_back(edge.from);
    projection.to_station.push_back(edge.to);
    projection.distance_m.push_back(edge.distance_m);
    projection.bidirectional.push_back(edge.bidirectional);
    projection.enabled.push_back(edge.enabled);
  }
  projection.message = "loaded " + std::to_string(catalog.edges.size()) + " route edge(s)";
  return projection;
}

}  // namespace robot_tasks
