#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_tasks/facility_catalog.hpp"

namespace robot_tasks {

struct ResourceReservation {
  std::string holder_id;
  std::string mission_id;
  std::string status = "reserved";
  bool hold_after_completion = true;
};

using FacilityReservationMap = std::unordered_map<std::string, ResourceReservation>;
using MissionResourceMap = std::unordered_map<std::string, std::string>;

struct FacilityReservationResult {
  bool success = false;
  std::string message;
  std::string resource_id;
  std::string holder_id;
  std::string status;
};

struct FacilityResourceListRequest {
  bool include_disabled = false;
  std::string resource_type;
};

struct FacilityResourceListProjection {
  bool success = true;
  std::vector<std::string> resource_ids;
  std::vector<std::string> resource_types;
  std::vector<std::string> station_ids;
  std::vector<bool> enabled;
  std::vector<bool> available;
  std::vector<std::string> holder_ids;
  std::vector<std::string> status;
  std::string message;
};

struct FacilityResourceStateProjection {
  bool success = true;
  std::string resource_id;
  std::string station_id;
  bool available = false;
  std::string holder_id;
  std::string status;
  std::string message;
};

bool FacilityResourceAvailable(
    const FacilityResource& resource, const FacilityReservationMap& reservations);
std::string FacilityResourceStatus(
    const FacilityResource& resource, const FacilityReservationMap& reservations);
FacilityResourceListProjection BuildFacilityResourceListProjection(
    const FacilityCatalog& catalog,
    const FacilityReservationMap& reservations,
    const FacilityResourceListRequest& request);
FacilityResourceStateProjection BuildFacilityResourceStateProjection(
    const FacilityResource& resource,
    const FacilityReservationMap& reservations,
    const std::string& resource_label);
std::vector<std::string> ReservedResourceIds(const FacilityReservationMap& reservations);
std::size_t ReleaseResourcesForHolder(
    FacilityReservationMap& reservations, const std::string& holder_id);
FacilityReservationResult ReserveFacilityResource(
    FacilityReservationMap& reservations, const FacilityResource& resource,
    const std::string& holder_id, const std::string& mission_id,
    bool release_existing_for_holder);
FacilityReservationResult ReleaseFacilityResource(
    FacilityReservationMap& reservations, const std::string& resource_id,
    const std::string& holder_id);
bool MarkResourceOccupiedForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id);
bool ReleaseResourceForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
