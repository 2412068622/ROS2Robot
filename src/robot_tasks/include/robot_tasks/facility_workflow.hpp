#pragma once

#include <string>

#include "robot_tasks/facility_catalog.hpp"
#include "robot_tasks/facility_reservation.hpp"

namespace robot_tasks {

struct FacilityActionResourceSelection {
  bool success = false;
  const FacilityResource* resource = nullptr;
  std::string message;
};

struct LiftSessionResult {
  bool success = false;
  std::string session_id;
  std::string lift_id;
  std::string status;
  std::string message;
};

bool FacilityActionSupported(const std::string& resource_type, const std::string& action);

FacilityActionResourceSelection SelectFacilityActionResource(
    const FacilityCatalog& catalog, const FacilityReservationMap& reservations,
    const std::string& requested_resource_id, const std::string& requested_resource_type);

std::string BuildFacilityActionMissionId(
    const std::string& request_id, const std::string& resource_id, const std::string& action);

void ReserveFacilityActionResource(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const FacilityResource& resource, const std::string& mission_id, bool hold_after_action);

std::string BuildLiftSessionId(const std::string& requested_session_id, const std::string& lift_id);

std::string BuildLiftSessionHolderId(
    const std::string& requester_id, const std::string& session_id);

LiftSessionResult ReserveLiftSessionResource(
    FacilityReservationMap& reservations, const FacilityResource& lift,
    const std::string& session_id, const std::string& holder_id,
    bool release_existing_for_holder);

LiftSessionResult ReleaseLiftSessionResource(
    FacilityReservationMap& reservations, const std::string& lift_id,
    const std::string& session_id);

}  // namespace robot_tasks
