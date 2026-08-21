#pragma once

#include <string>
#include <unordered_map>

#include "robot_tasks/facility_reservation.hpp"
#include "robot_tasks/mission_preflight.hpp"

namespace robot_tasks {

struct DockingRuntimeState {
  std::string dock_id;
  std::string mission_id;
  std::string state = "AVAILABLE";
  bool simulate_contact_success = true;
  int contact_attempts = 0;
};

using DockingStateMap = std::unordered_map<std::string, DockingRuntimeState>;
using DockingMissionMap = std::unordered_map<std::string, std::string>;

struct DockingTransition {
  bool changed = false;
  std::string mission_id;
  std::string dock_id;
  std::string previous_state;
  std::string state;
  std::string event_state;
  std::string event_message;
  bool event_recoverable = true;
};

struct DockReturnRequest {
  bool dock_profile_available = false;
  std::string load_error_message;
  bool dock_already_requested_or_running = false;
  bool mission_active = false;
};

struct DockReturnDecision {
  bool response_success = false;
  std::string response_message;
  bool set_error_state = false;
  std::string error_message;
  bool queue_dock_mission = false;
  bool cancel_active_mission = false;
  bool dispatch_dock_mission = false;
  std::string branch = "load_failed";
};

struct UndockingRequest {
  std::string dock_id;
  std::string charger_resource_id;
  std::string request_id;
  bool release_charger = false;
};

struct UndockingDecision {
  bool success = true;
  bool mutate_state = true;
  std::string mission_id;
  std::string dock_id;
  std::string previous_state;
  std::string state = "UNDOCKED";
  std::string message;
  bool release_charger = false;
  std::string release_mission_id;
};

struct DockingRequestGateRequest {
  std::string dock_id;
  bool dock_enabled = true;
  bool dock_available = true;
  std::string dock_state = "AVAILABLE";
  bool dock_references_valid = true;
  std::string dock_reference_message;
  std::string mission_id;
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

struct DockingRequestGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string state;
  std::string message;
};

DockingTransition CompleteDockingSubflow(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

DockingTransition StageDockingContactRetry(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

DockingTransition MarkDockingSubflowTerminal(
    DockingStateMap& dock_states, DockingMissionMap& dock_by_mission,
    const std::string& mission_id, const std::string& state, const std::string& message);

DockReturnDecision PlanDockReturn(const DockReturnRequest& request);

DockingRequestGateDecision PlanDockingRequestGate(
    const DockingRequestGateRequest& request);

UndockingDecision PlanUndocking(
    const UndockingRequest& request, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations);

void ApplyUndockingDecision(
    const UndockingDecision& decision, const std::string& charger_resource_id,
    DockingStateMap& dock_states, FacilityReservationMap& reservations,
    MissionResourceMap& resource_by_mission);

}  // namespace robot_tasks
