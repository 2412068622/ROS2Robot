#pragma once

#include <string>

#include "robot_tasks/mission_resource_cleanup.hpp"

namespace robot_tasks {

struct MissionResourceCleanupBehaviorTreeInput {
  FacilityReservationMap& facility_reservations;
  MissionResourceMap& facility_resource_by_mission;
  RouteLockMap& route_locks;
  RouteLockReservationMap& route_locks_by_mission;
  std::string mission_id;
};

struct MissionResourceCleanupBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionResourceCleanupResult cleanup;
};

std::string DefaultMissionResourceCleanupBehaviorTreeXml();
MissionResourceCleanupBehaviorTreeResult TickMissionResourceCleanupBehaviorTree(
    MissionResourceCleanupBehaviorTreeInput input);

}  // namespace robot_tasks
