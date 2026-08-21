#include "robot_tasks/mission_profile_builders.hpp"

#include <cstddef>
#include <string>

namespace robot_tasks {

MissionProfile BuildTwoWaypointProfile(
    const std::string& mission_id, const std::string& frame_id, const double pickup_x,
    const double pickup_y, const double pickup_yaw, const double dropoff_x,
    const double dropoff_y, const double dropoff_yaw) {
  MissionProfile profile;
  profile.mission_id = mission_id;
  profile.frame_id = frame_id.empty() ? "map" : frame_id;
  profile.loop = false;
  profile.waypoints = {
      MissionWaypoint{pickup_x, pickup_y, pickup_yaw},
      MissionWaypoint{dropoff_x, dropoff_y, dropoff_yaw}};
  return profile;
}

MissionProfile BuildStationWaypointProfile(const std::string& mission_id, const Station& station) {
  MissionProfile profile;
  profile.mission_id = mission_id;
  profile.frame_id = station.frame_id.empty() ? "map" : station.frame_id;
  profile.loop = false;
  profile.waypoints = {MissionWaypoint{station.x, station.y, station.yaw}};
  return profile;
}

std::optional<StationPair> ResolveStationPair(
    const StationCatalog& catalog, const std::string& pickup_station,
    const std::string& dropoff_station, std::string* message) {
  const auto* pickup = FindStation(catalog, pickup_station);
  if (pickup == nullptr) {
    if (message != nullptr) {
      *message = "unknown pickup station: " + pickup_station;
    }
    return std::nullopt;
  }
  const auto* dropoff = FindStation(catalog, dropoff_station);
  if (dropoff == nullptr) {
    if (message != nullptr) {
      *message = "unknown dropoff station: " + dropoff_station;
    }
    return std::nullopt;
  }
  if (pickup->frame_id != dropoff->frame_id) {
    if (message != nullptr) {
      *message = "pickup and dropoff stations use different frames: " + pickup->frame_id +
                 " vs " + dropoff->frame_id;
    }
    return std::nullopt;
  }
  return StationPair{*pickup, *dropoff};
}

std::optional<std::vector<StationSequenceLeg>> BuildStationSequenceProfiles(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    const std::string& base_id, std::string* message) {
  if (station_ids.size() < 2U) {
    if (message != nullptr) {
      *message = "station sequence requires at least two stations";
    }
    return std::nullopt;
  }

  std::vector<StationSequenceLeg> legs;
  legs.reserve(station_ids.size() - 1U);
  for (std::size_t index = 1; index < station_ids.size(); ++index) {
    const auto pair = ResolveStationPair(catalog, station_ids[index - 1], station_ids[index], message);
    if (!pair.has_value()) {
      return std::nullopt;
    }
    const auto route_distance =
        EstimateRouteDistance(catalog, station_ids[index - 1], station_ids[index]);
    if (!route_distance.has_value()) {
      if (message != nullptr) {
        *message = "no enabled station route from " + station_ids[index - 1] + " to " +
                   station_ids[index];
      }
      return std::nullopt;
    }
    const auto mission_id = base_id + "_leg_" + std::to_string(index);
    auto profile = BuildTwoWaypointProfile(
        mission_id, pair->pickup.frame_id, pair->pickup.x, pair->pickup.y, pair->pickup.yaw,
        pair->dropoff.x, pair->dropoff.y, pair->dropoff.yaw);
    if (!ValidateMissionProfile(profile, message)) {
      if (message != nullptr) {
        *message = "invalid station sequence leg " + mission_id + ": " + *message;
      }
      return std::nullopt;
    }
    legs.push_back(StationSequenceLeg{profile, station_ids[index - 1], station_ids[index]});
  }
  return legs;
}

std::optional<MissionProfile> BuildStationSequenceEstimateProfile(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    std::string* message) {
  if (station_ids.size() < 2U) {
    if (message != nullptr) {
      *message = "station sequence requires at least two stations";
    }
    return std::nullopt;
  }
  MissionProfile profile;
  profile.mission_id = "estimate_station_sequence";
  for (const auto& station_id : station_ids) {
    const auto* station = FindStation(catalog, station_id);
    if (station == nullptr) {
      if (message != nullptr) {
        *message = "unknown station: " + station_id;
      }
      return std::nullopt;
    }
    if (profile.waypoints.empty()) {
      profile.frame_id = station->frame_id.empty() ? "map" : station->frame_id;
    } else if (profile.frame_id != station->frame_id) {
      if (message != nullptr) {
        *message = "station sequence contains mixed frames";
      }
      return std::nullopt;
    }
    profile.waypoints.push_back(MissionWaypoint{station->x, station->y, station->yaw});
  }
  if (!ValidateMissionProfile(profile, message)) {
    return std::nullopt;
  }
  return profile;
}

}  // namespace robot_tasks
