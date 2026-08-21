#include "robot_tasks/traffic_reservation.hpp"

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace robot_tasks {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

std::string RouteLockId(const std::string& from, const std::string& to) {
  return from <= to ? "route_edge:" + from + "__" + to : "route_edge:" + to + "__" + from;
}

std::string RouteNodeLockId(const std::string& station_id) {
  return "route_node:" + station_id;
}

std::optional<std::pair<std::string, std::string>> ParseRouteLock(
    const std::string& resource_id) {
  const std::string prefix = "route_edge:";
  if (!StartsWith(resource_id, prefix)) {
    return std::nullopt;
  }
  const auto separator = resource_id.find("__", prefix.size());
  if (separator == std::string::npos) {
    return std::nullopt;
  }
  const auto first = resource_id.substr(prefix.size(), separator - prefix.size());
  const auto second = resource_id.substr(separator + 2);
  if (first.empty() || second.empty()) {
    return std::nullopt;
  }
  return std::make_pair(first, second);
}

bool IsTrafficBlockOwner(const std::string& owner_id) {
  return StartsWith(owner_id, "traffic_block:");
}

std::string TrafficBlockOwnerId(const std::string& reason) {
  return "traffic_block:" + reason;
}

TrafficReservationDecision PlanTrafficBlock(
    const TrafficBlockRequest& request, const RouteLockMap& route_locks) {
  TrafficReservationDecision decision;
  decision.resource_id = RouteLockId(request.from_station, request.to_station);
  if (!request.stations_known) {
    decision.message = "unknown station in route block request: " + request.from_station +
                       " -> " + request.to_station;
    return decision;
  }
  if (!request.route_exists) {
    decision.message =
        "no enabled station route to block: " + request.from_station + " -> " + request.to_station;
    return decision;
  }
  const auto existing = route_locks.find(decision.resource_id);
  if (existing != route_locks.end()) {
    decision.message =
        "route resource already reserved: " + decision.resource_id + " by " + existing->second;
    return decision;
  }
  decision.success = true;
  decision.mutate_state = true;
  decision.owner_id = TrafficBlockOwnerId(request.reason);
  decision.message = "blocked route " + request.from_station + " -> " + request.to_station;
  return decision;
}

TrafficReservationDecision PlanTrafficUnblock(
    const std::string& from_station, const std::string& to_station,
    const RouteLockMap& route_locks) {
  TrafficReservationDecision decision;
  decision.resource_id = RouteLockId(from_station, to_station);
  const auto lock = route_locks.find(decision.resource_id);
  if (lock == route_locks.end()) {
    decision.message = "route resource not found: " + decision.resource_id;
    return decision;
  }
  decision.owner_id = lock->second;
  if (!IsTrafficBlockOwner(lock->second)) {
    decision.message = "route resource is reserved by mission: " + lock->second;
    return decision;
  }
  decision.success = true;
  decision.mutate_state = true;
  decision.message = "unblocked route " + from_station + " -> " + to_station;
  return decision;
}

TrafficClearDecision PlanTrafficReservationClear(
    const std::string& resource_id, const RouteLockMap& route_locks) {
  TrafficClearDecision decision;
  decision.resource_id = resource_id;
  const auto lock = route_locks.find(resource_id);
  if (lock == route_locks.end()) {
    decision.message = "traffic reservation not found: " + resource_id;
    return decision;
  }
  decision.success = true;
  decision.owner_id = lock->second;
  if (IsTrafficBlockOwner(lock->second)) {
    decision.erase_resource = true;
    decision.message = "cleared blocked route " + resource_id;
    return decision;
  }
  decision.release_mission_id = lock->second;
  decision.message = "cleared mission route reservation " + resource_id;
  return decision;
}

TrafficDeadlockDecision PlanTrafficDeadlockDetection(const RouteLockMap& route_locks) {
  TrafficDeadlockDecision decision;
  std::unordered_map<std::string, std::vector<std::string>> station_to_missions;
  std::unordered_map<std::string, std::vector<std::string>> station_to_resources;
  for (const auto& [resource_id, owner_id] : route_locks) {
    if (IsTrafficBlockOwner(owner_id)) {
      continue;
    }
    const auto lock = ParseRouteLock(resource_id);
    if (!lock.has_value()) {
      continue;
    }
    station_to_missions[lock->first].push_back(owner_id);
    station_to_missions[lock->second].push_back(owner_id);
    station_to_resources[lock->first].push_back(resource_id);
    station_to_resources[lock->second].push_back(resource_id);
  }

  std::unordered_set<std::string> conflict_missions;
  std::unordered_set<std::string> conflict_resources;
  std::unordered_set<std::string> conflict_stations;
  for (const auto& [station_id, mission_ids] : station_to_missions) {
    std::unordered_set<std::string> unique_missions(mission_ids.begin(), mission_ids.end());
    if (unique_missions.size() < 2U) {
      continue;
    }
    conflict_stations.insert(station_id);
    std::vector<std::string> ordered_missions(unique_missions.begin(), unique_missions.end());
    std::sort(ordered_missions.begin(), ordered_missions.end());
    for (const auto& mission_id : ordered_missions) {
      conflict_missions.insert(mission_id);
    }
    const auto resource_it = station_to_resources.find(station_id);
    if (resource_it != station_to_resources.end()) {
      for (const auto& resource_id : resource_it->second) {
        conflict_resources.insert(resource_id);
      }
    }
    std::ostringstream desc;
    desc << "station " << station_id << " shared by missions ";
    for (std::size_t index = 0; index < ordered_missions.size(); ++index) {
      if (index > 0) {
        desc << ", ";
      }
      desc << ordered_missions[index];
    }
    decision.descriptions.push_back(desc.str());
  }

  decision.deadlocked = !conflict_stations.empty();
  decision.mission_ids.assign(conflict_missions.begin(), conflict_missions.end());
  std::sort(decision.mission_ids.begin(), decision.mission_ids.end());
  decision.conflict_resource_ids.assign(conflict_resources.begin(), conflict_resources.end());
  std::sort(decision.conflict_resource_ids.begin(), decision.conflict_resource_ids.end());
  decision.conflict_station_ids.assign(conflict_stations.begin(), conflict_stations.end());
  std::sort(decision.conflict_station_ids.begin(), decision.conflict_station_ids.end());
  if (decision.deadlocked) {
    decision.message = "traffic deadlock candidates detected";
  }
  return decision;
}

std::vector<std::string> BuildRouteLockIds(
    const std::vector<std::string>& station_path, const bool include_intersection_locks) {
  std::vector<std::string> lock_ids;
  if (include_intersection_locks) {
    for (const auto& station_id : station_path) {
      lock_ids.push_back(RouteNodeLockId(station_id));
    }
  }
  for (std::size_t index = 1; index < station_path.size(); ++index) {
    lock_ids.push_back(RouteLockId(station_path[index - 1], station_path[index]));
  }
  return lock_ids;
}

bool ReserveRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
    std::string* message) {
  for (const auto& lock_id : route_lock_ids) {
    const auto existing = route_locks.find(lock_id);
    if (existing != route_locks.end() && existing->second != mission_id) {
      if (message != nullptr) {
        *message = "route resource locked: " + lock_id + " by " + existing->second;
      }
      return false;
    }
  }
  for (const auto& lock_id : route_lock_ids) {
    route_locks[lock_id] = mission_id;
  }
  route_locks_by_mission[mission_id] = RouteLockReservation{mission_id, route_lock_ids};
  if (message != nullptr) {
    *message = "reserved " + std::to_string(route_lock_ids.size()) + " route lock(s)";
  }
  return true;
}

bool ReleaseRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id) {
  bool released = false;
  const auto reservation = route_locks_by_mission.find(mission_id);
  if (reservation != route_locks_by_mission.end()) {
    for (const auto& lock_id : reservation->second.route_lock_ids) {
      const auto lock = route_locks.find(lock_id);
      if (lock != route_locks.end() && lock->second == mission_id) {
        route_locks.erase(lock);
        released = true;
      }
    }
    route_locks_by_mission.erase(reservation);
  }
  for (auto lock = route_locks.begin(); lock != route_locks.end();) {
    if (lock->second == mission_id) {
      lock = route_locks.erase(lock);
      released = true;
    } else {
      ++lock;
    }
  }
  return released;
}

}  // namespace robot_tasks
