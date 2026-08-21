#include "robot_tasks/facility_workflow.hpp"

namespace robot_tasks {

bool FacilityActionSupported(const std::string& resource_type, const std::string& action) {
  if (resource_type == "door") {
    return action == "open" || action == "close" || action == "pass";
  }
  if (resource_type == "elevator") {
    return action == "call" || action == "enter" || action == "exit" || action == "ride";
  }
  if (resource_type == "charger") {
    return action == "reserve" || action == "release";
  }
  return action == "reserve" || action == "release" || action == "use";
}

FacilityActionResourceSelection SelectFacilityActionResource(
    const FacilityCatalog& catalog, const FacilityReservationMap& reservations,
    const std::string& requested_resource_id, const std::string& requested_resource_type) {
  FacilityActionResourceSelection selection;
  if (!requested_resource_id.empty()) {
    const auto* resource = FindFacilityResource(catalog, requested_resource_id);
    if (resource == nullptr) {
      selection.message = "unknown facility resource: " + requested_resource_id;
      return selection;
    }
    if (!requested_resource_type.empty() && resource->type != requested_resource_type) {
      selection.message = "facility resource type mismatch: " + requested_resource_id;
      return selection;
    }
    if (!FacilityResourceAvailable(*resource, reservations)) {
      selection.message = "facility resource unavailable: " + requested_resource_id;
      return selection;
    }
    selection.success = true;
    selection.resource = resource;
    return selection;
  }

  if (requested_resource_type.empty()) {
    selection.message = "resource_id or resource_type is required";
    return selection;
  }
  const auto* resource =
      FindFirstAvailableResource(catalog, requested_resource_type, ReservedResourceIds(reservations));
  if (resource == nullptr) {
    selection.message = "no available facility resource of type: " + requested_resource_type;
    return selection;
  }
  selection.success = true;
  selection.resource = resource;
  return selection;
}

std::string BuildFacilityActionMissionId(
    const std::string& request_id, const std::string& resource_id, const std::string& action) {
  return request_id.empty() ? "facility_action_" + resource_id + "_" + action
                            : "facility_action_" + request_id;
}

void ReserveFacilityActionResource(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const FacilityResource& resource, const std::string& mission_id,
    const bool hold_after_action) {
  reservations[resource.id] =
      ResourceReservation{mission_id, mission_id, "reserved", hold_after_action};
  resource_by_mission[mission_id] = resource.id;
}

std::string BuildLiftSessionId(
    const std::string& requested_session_id, const std::string& lift_id) {
  return requested_session_id.empty() ? "lift_" + lift_id : requested_session_id;
}

std::string BuildLiftSessionHolderId(
    const std::string& requester_id, const std::string& session_id) {
  return requester_id.empty() ? session_id : requester_id;
}

LiftSessionResult ReserveLiftSessionResource(
    FacilityReservationMap& reservations, const FacilityResource& lift,
    const std::string& session_id, const std::string& holder_id,
    const bool release_existing_for_holder) {
  LiftSessionResult result;
  result.session_id = session_id;
  result.lift_id = lift.id;

  if (release_existing_for_holder) {
    ReleaseResourcesForHolder(reservations, holder_id);
  }
  const auto existing = reservations.find(lift.id);
  if (existing != reservations.end() && existing->second.holder_id != holder_id) {
    result.status = existing->second.status;
    result.message = "lift already reserved by " + existing->second.holder_id;
    return result;
  }

  reservations[lift.id] = ResourceReservation{holder_id, session_id, "lift_session", true};
  result.success = true;
  result.status = "lift_session";
  result.message = "reserved lift " + lift.id + " for session " + session_id;
  return result;
}

LiftSessionResult ReleaseLiftSessionResource(
    FacilityReservationMap& reservations, const std::string& lift_id,
    const std::string& session_id) {
  LiftSessionResult result;
  result.lift_id = lift_id;
  result.session_id = session_id;

  const auto existing = reservations.find(lift_id);
  if (existing == reservations.end()) {
    result.success = true;
    result.status = "available";
    result.message = "lift already available: " + lift_id;
    return result;
  }
  if (!session_id.empty() && existing->second.mission_id != session_id &&
      existing->second.holder_id != session_id) {
    result.status = existing->second.status;
    result.message = "lift held by " + existing->second.holder_id;
    return result;
  }

  reservations.erase(existing);
  result.success = true;
  result.status = "available";
  result.message = "released lift session " + session_id;
  return result;
}

}  // namespace robot_tasks
