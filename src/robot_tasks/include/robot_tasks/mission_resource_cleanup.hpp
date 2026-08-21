#pragma once

#include <string>

#include "robot_tasks/facility_reservation.hpp"
#include "robot_tasks/traffic_reservation.hpp"

namespace robot_tasks {

struct MissionResourceCleanupResult {
  bool facility_resource_released = false;
  bool route_locks_released = false;
};

MissionResourceCleanupResult ReleaseMissionResources(
    FacilityReservationMap& facility_reservations, MissionResourceMap& facility_resource_by_mission,
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
