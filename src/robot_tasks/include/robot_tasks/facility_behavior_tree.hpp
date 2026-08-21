#pragma once

#include <string>

#include "robot_tasks/facility_workflow.hpp"

namespace robot_tasks {

struct FacilityActionBehaviorTreeInput {
  FacilityCatalog catalog;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  std::string request_id;
  std::string resource_id;
  std::string resource_type;
  std::string action = "reserve";
  bool hold_after_action = true;
};

struct FacilityActionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::string mission_id;
  std::string resource_id;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
};

struct LiftSessionBehaviorTreeInput {
  FacilityReservationMap reservations;
  FacilityResource lift;
  std::string session_id;
  std::string requester_id;
  bool release_existing_for_holder = true;
  bool release = false;
};

struct LiftSessionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  LiftSessionResult lift_result;
  FacilityReservationMap reservations;
};

std::string DefaultFacilityActionBehaviorTreeXml();
FacilityActionBehaviorTreeResult TickFacilityActionBehaviorTree(
    const FacilityActionBehaviorTreeInput& input);

std::string DefaultLiftSessionBehaviorTreeXml();
LiftSessionBehaviorTreeResult TickLiftSessionBehaviorTree(
    const LiftSessionBehaviorTreeInput& input);

}  // namespace robot_tasks
