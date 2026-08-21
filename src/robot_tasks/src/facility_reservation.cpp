#include "robot_tasks/facility_reservation.hpp"

namespace robot_tasks {

bool FacilityResourceAvailable(
    const FacilityResource& resource, const FacilityReservationMap& reservations) {
  if (!resource.enabled) {
    return false;
  }
  return !resource.exclusive || reservations.find(resource.id) == reservations.end();
}

std::string FacilityResourceStatus(
    const FacilityResource& resource, const FacilityReservationMap& reservations) {
  if (!resource.enabled) {
    return "disabled";
  }
  const auto reservation = reservations.find(resource.id);
  return reservation == reservations.end() ? "available" : reservation->second.status;
}

FacilityResourceListProjection BuildFacilityResourceListProjection(
    const FacilityCatalog& catalog,
    const FacilityReservationMap& reservations,
    const FacilityResourceListRequest& request) {
  FacilityResourceListProjection projection;
  for (const auto& resource : catalog.resources) {
    if (!request.include_disabled && !resource.enabled) {
      continue;
    }
    if (!request.resource_type.empty() && resource.type != request.resource_type) {
      continue;
    }
    projection.resource_ids.push_back(resource.id);
    projection.resource_types.push_back(resource.type);
    projection.station_ids.push_back(resource.station_id);
    projection.enabled.push_back(resource.enabled);
    projection.available.push_back(FacilityResourceAvailable(resource, reservations));
    const auto reservation = reservations.find(resource.id);
    projection.holder_ids.push_back(
        reservation == reservations.end() ? "" : reservation->second.holder_id);
    projection.status.push_back(FacilityResourceStatus(resource, reservations));
  }
  projection.message =
      "loaded " + std::to_string(projection.resource_ids.size()) + " facility resource(s)";
  return projection;
}

FacilityResourceStateProjection BuildFacilityResourceStateProjection(
    const FacilityResource& resource,
    const FacilityReservationMap& reservations,
    const std::string& resource_label) {
  FacilityResourceStateProjection projection;
  projection.resource_id = resource.id;
  projection.station_id = resource.station_id;
  projection.available = FacilityResourceAvailable(resource, reservations);
  const auto reservation = reservations.find(resource.id);
  projection.holder_id = reservation == reservations.end() ? "" : reservation->second.holder_id;
  projection.status = FacilityResourceStatus(resource, reservations);
  projection.message = "loaded " + resource_label + " state " + resource.id;
  return projection;
}

std::vector<std::string> ReservedResourceIds(const FacilityReservationMap& reservations) {
  std::vector<std::string> ids;
  ids.reserve(reservations.size());
  for (const auto& [resource_id, reservation] : reservations) {
    (void)reservation;
    ids.push_back(resource_id);
  }
  return ids;
}

std::size_t ReleaseResourcesForHolder(
    FacilityReservationMap& reservations, const std::string& holder_id) {
  std::size_t released = 0U;
  for (auto it = reservations.begin(); it != reservations.end();) {
    if (it->second.holder_id == holder_id) {
      it = reservations.erase(it);
      ++released;
    } else {
      ++it;
    }
  }
  return released;
}

FacilityReservationResult ReserveFacilityResource(
    FacilityReservationMap& reservations, const FacilityResource& resource,
    const std::string& holder_id, const std::string& mission_id,
    const bool release_existing_for_holder) {
  FacilityReservationResult result;
  result.resource_id = resource.id;
  result.holder_id = holder_id;

  if (!resource.enabled) {
    result.status = "disabled";
    result.message = "facility resource disabled: " + resource.id;
    return result;
  }
  if (release_existing_for_holder) {
    ReleaseResourcesForHolder(reservations, holder_id);
  }
  const auto existing = reservations.find(resource.id);
  if (resource.exclusive && existing != reservations.end() &&
      existing->second.holder_id != holder_id) {
    result.holder_id = existing->second.holder_id;
    result.status = existing->second.status;
    result.message = "facility resource already reserved: " + resource.id;
    return result;
  }

  reservations[resource.id] = ResourceReservation{holder_id, mission_id, "reserved"};
  result.success = true;
  result.status = "reserved";
  result.message = "reserved facility resource " + resource.id + " for " + holder_id;
  return result;
}

FacilityReservationResult ReleaseFacilityResource(
    FacilityReservationMap& reservations, const std::string& resource_id,
    const std::string& holder_id) {
  FacilityReservationResult result;
  result.resource_id = resource_id;
  const auto existing = reservations.find(resource_id);
  if (existing == reservations.end()) {
    result.success = true;
    result.status = "available";
    result.message = "facility resource already available: " + resource_id;
    return result;
  }
  if (!holder_id.empty() && existing->second.holder_id != holder_id) {
    result.holder_id = existing->second.holder_id;
    result.status = existing->second.status;
    result.message = "facility resource held by " + existing->second.holder_id;
    return result;
  }

  reservations.erase(existing);
  result.success = true;
  result.status = "available";
  result.message = "released facility resource " + resource_id;
  return result;
}

bool MarkResourceOccupiedForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id) {
  const auto resource_it = resource_by_mission.find(mission_id);
  if (resource_it == resource_by_mission.end()) {
    return false;
  }
  const auto reservation_it = reservations.find(resource_it->second);
  if (reservation_it == reservations.end()) {
    return false;
  }
  if (reservation_it->second.hold_after_completion) {
    reservation_it->second.status = "occupied";
  } else {
    reservations.erase(reservation_it);
    resource_by_mission.erase(resource_it);
  }
  return true;
}

bool ReleaseResourceForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id) {
  const auto resource_it = resource_by_mission.find(mission_id);
  if (resource_it == resource_by_mission.end()) {
    return false;
  }
  const auto reservation_it = reservations.find(resource_it->second);
  if (reservation_it != reservations.end() && reservation_it->second.mission_id == mission_id) {
    reservations.erase(reservation_it);
  }
  resource_by_mission.erase(resource_it);
  return true;
}

}  // namespace robot_tasks
