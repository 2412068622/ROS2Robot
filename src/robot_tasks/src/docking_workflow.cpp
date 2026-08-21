#include "robot_tasks/docking_workflow.hpp"

namespace robot_tasks {
namespace {

DockingTransition NoChange(const std::string& mission_id) {
  DockingTransition transition;
  transition.mission_id = mission_id;
  return transition;
}

std::string PreviousDockState(const DockingStateMap& dock_states, const std::string& dock_id) {
  const auto state_it = dock_states.find(dock_id);
  return state_it == dock_states.end() ? "APPROACHING" : state_it->second.state;
}

void UpdateReservedResourceStatus(
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id, const std::string& status) {
  const auto resource_it = resource_by_mission.find(mission_id);
  if (resource_it == resource_by_mission.end()) {
    return;
  }
  const auto reservation_it = reservations.find(resource_it->second);
  if (reservation_it != reservations.end()) {
    reservation_it->second.status = status;
  }
}

DockingTransition Transition(
    const std::string& mission_id, const std::string& dock_id, const std::string& previous_state,
    const std::string& state, const std::string& event_state,
    const std::string& event_message, const bool event_recoverable) {
  DockingTransition transition;
  transition.changed = true;
  transition.mission_id = mission_id;
  transition.dock_id = dock_id;
  transition.previous_state = previous_state;
  transition.state = state;
  transition.event_state = event_state;
  transition.event_message = event_message;
  transition.event_recoverable = event_recoverable;
  return transition;
}

}  // namespace

DockingTransition CompleteDockingSubflow(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id) {
  const auto dock_it = dock_by_mission.find(mission_id);
  if (dock_it == dock_by_mission.end()) {
    return NoChange(mission_id);
  }

  const auto dock_id = dock_it->second;
  const auto previous = PreviousDockState(dock_states, dock_id);
  dock_states[dock_id] = DockingRuntimeState{dock_id, mission_id, "CHARGING"};
  UpdateReservedResourceStatus(reservations, resource_by_mission, mission_id, "CHARGING");
  return Transition(
      mission_id, dock_id, previous, "CHARGING", "CHARGING",
      "docked and charging at " + dock_id, true);
}

DockingTransition StageDockingContactRetry(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id) {
  const auto dock_it = dock_by_mission.find(mission_id);
  if (dock_it == dock_by_mission.end()) {
    return NoChange(mission_id);
  }

  const auto dock_id = dock_it->second;
  auto state_it = dock_states.find(dock_id);
  if (state_it == dock_states.end() || state_it->second.simulate_contact_success ||
      state_it->second.contact_attempts > 0) {
    return NoChange(mission_id);
  }

  const auto previous = state_it->second.state;
  state_it->second.state = "FAILED";
  state_it->second.contact_attempts += 1;
  state_it->second.simulate_contact_success = true;
  UpdateReservedResourceStatus(
      reservations, resource_by_mission, mission_id, "DOCKING_CONTACT_FAILED");
  return Transition(
      mission_id, dock_id, previous, "FAILED", "DOCKING_CONTACT_FAILED",
      "docking contact failed at " + dock_id + ", scheduling retry", true);
}

DockingTransition MarkDockingSubflowTerminal(
    DockingStateMap& dock_states, DockingMissionMap& dock_by_mission,
    const std::string& mission_id, const std::string& state, const std::string& message) {
  const auto dock_it = dock_by_mission.find(mission_id);
  if (dock_it == dock_by_mission.end()) {
    return NoChange(mission_id);
  }

  const auto dock_id = dock_it->second;
  const auto previous = PreviousDockState(dock_states, dock_id);
  dock_states[dock_id] = DockingRuntimeState{dock_id, mission_id, state};
  dock_by_mission.erase(dock_it);
  return Transition(mission_id, dock_id, previous, state, state, message, state != "FAILED");
}

DockReturnDecision PlanDockReturn(const DockReturnRequest& request) {
  DockReturnDecision decision;
  if (!request.dock_profile_available) {
    decision.branch = "load_failed";
    decision.set_error_state = true;
    decision.error_message = request.load_error_message;
    decision.response_success = false;
    decision.response_message = request.load_error_message;
    return decision;
  }
  if (request.dock_already_requested_or_running) {
    decision.branch = "already_requested";
    decision.response_success = true;
    decision.response_message = "return to dock already active or queued";
    return decision;
  }
  if (request.mission_active) {
    decision.branch = "queue_then_cancel_active";
    decision.response_success = true;
    decision.response_message = "return to dock queued and active mission cancel requested";
    decision.queue_dock_mission = true;
    decision.cancel_active_mission = true;
    return decision;
  }

  decision.branch = "dispatch_now";
  decision.dispatch_dock_mission = true;
  return decision;
}

DockingRequestGateDecision PlanDockingRequestGate(
    const DockingRequestGateRequest& request) {
  DockingRequestGateDecision decision;
  decision.mission_id = request.mission_id;

  if (!request.dock_enabled || !request.dock_available || !request.dock_references_valid) {
    decision.state = request.dock_state;
    decision.message = !request.dock_enabled ? "dock disabled: " + request.dock_id
                                             : request.dock_reference_message;
    if (decision.message.empty()) {
      decision.message = "dock unavailable: " + request.dock_id;
    }
    return decision;
  }

  if (request.mission_already_active_or_queued) {
    decision.state = request.dock_state;
    decision.message = "docking mission already active or queued: " + request.mission_id;
    return decision;
  }

  if (!request.preflight.allowed) {
    decision.state = "REJECTED";
    decision.message = "docking preflight rejected: " + request.preflight.message;
    return decision;
  }

  decision.accepted = true;
  decision.state = "APPROACHING";
  return decision;
}

UndockingDecision PlanUndocking(
    const UndockingRequest& request, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations) {
  UndockingDecision decision;
  decision.dock_id = request.dock_id;
  const auto state_it = dock_states.find(request.dock_id);
  if (state_it != dock_states.end()) {
    decision.previous_state = state_it->second.state;
    decision.release_mission_id = state_it->second.mission_id;
  } else {
    const auto reservation_it = reservations.find(request.charger_resource_id);
    decision.previous_state =
        reservation_it == reservations.end() ? "AVAILABLE" : reservation_it->second.status;
    decision.release_mission_id =
        reservation_it == reservations.end() ? "" : reservation_it->second.mission_id;
  }
  if (!request.request_id.empty()) {
    decision.mission_id = "undocking_request_" + request.request_id;
  } else if (!decision.release_mission_id.empty()) {
    decision.mission_id = decision.release_mission_id;
  } else {
    decision.mission_id = "undocking_request_" + request.dock_id;
  }
  decision.release_charger = request.release_charger;
  decision.message = "undocked from " + request.dock_id;
  return decision;
}

void ApplyUndockingDecision(
    const UndockingDecision& decision, const std::string& charger_resource_id,
    DockingStateMap& dock_states, FacilityReservationMap& reservations,
    MissionResourceMap& resource_by_mission) {
  if (!decision.mutate_state) {
    return;
  }
  dock_states[decision.dock_id] = DockingRuntimeState{decision.dock_id, decision.mission_id,
                                                       decision.state};
  if (!decision.release_charger) {
    return;
  }

  const auto reservation_it = reservations.find(charger_resource_id);
  if (reservation_it != reservations.end() &&
      reservation_it->second.mission_id == decision.release_mission_id) {
    reservations.erase(reservation_it);
  }
  for (auto it = resource_by_mission.begin(); it != resource_by_mission.end();) {
    if (it->first == decision.release_mission_id && it->second == charger_resource_id) {
      it = resource_by_mission.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace robot_tasks
