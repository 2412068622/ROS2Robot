#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_tasks {

struct RouteLockReservation {
  std::string mission_id;
  std::vector<std::string> route_lock_ids;
};

using RouteLockMap = std::unordered_map<std::string, std::string>;
using RouteLockReservationMap = std::unordered_map<std::string, RouteLockReservation>;

struct TrafficBlockRequest {
  std::string from_station;
  std::string to_station;
  std::string reason;
  bool stations_known = false;
  bool route_exists = false;
};

struct TrafficReservationDecision {
  bool success = false;
  bool mutate_state = false;
  std::string resource_id;
  std::string owner_id;
  std::string message;
};

struct TrafficClearDecision {
  bool success = false;
  bool erase_resource = false;
  std::string resource_id;
  std::string owner_id;
  std::string release_mission_id;
  std::string message;
};

struct TrafficDeadlockDecision {
  bool deadlocked = false;
  std::vector<std::string> mission_ids;
  std::vector<std::string> conflict_resource_ids;
  std::vector<std::string> conflict_station_ids;
  std::vector<std::string> descriptions;
  std::string message = "no traffic deadlock detected";
};

std::string RouteLockId(const std::string& from, const std::string& to);
std::string RouteNodeLockId(const std::string& station_id);
std::optional<std::pair<std::string, std::string>> ParseRouteLock(
    const std::string& resource_id);
bool IsTrafficBlockOwner(const std::string& owner_id);
std::string TrafficBlockOwnerId(const std::string& reason);
TrafficReservationDecision PlanTrafficBlock(
    const TrafficBlockRequest& request, const RouteLockMap& route_locks);
TrafficReservationDecision PlanTrafficUnblock(
    const std::string& from_station, const std::string& to_station,
    const RouteLockMap& route_locks);
TrafficClearDecision PlanTrafficReservationClear(
    const std::string& resource_id, const RouteLockMap& route_locks);
TrafficDeadlockDecision PlanTrafficDeadlockDetection(const RouteLockMap& route_locks);
std::vector<std::string> BuildRouteLockIds(
    const std::vector<std::string>& station_path, bool include_intersection_locks);
bool ReserveRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
    std::string* message);
bool ReleaseRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
