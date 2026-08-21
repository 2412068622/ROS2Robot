#include "robot_tasks/station_transport_workflow.hpp"

#include <utility>

#include "robot_tasks/traffic_reservation.hpp"

namespace robot_tasks {
namespace {

StationTransportTransition RejectTransition(
    const StationTransportRuntimeState& state,
    std::string message) {
  StationTransportTransition transition;
  transition.next_state = state;
  transition.message = std::move(message);
  return transition;
}

StationTransportTransition AcceptTransition(
    const StationTransportRuntimeState& state,
    std::string message) {
  StationTransportTransition transition;
  transition.accepted = true;
  transition.next_state = state;
  transition.message = std::move(message);
  return transition;
}

bool HasRequiredOrderIdentity(const StationTransportRuntimeState& state) {
  return !state.mission_id.empty() && !state.pickup_station_id.empty() &&
         !state.dropoff_station_id.empty();
}

}  // namespace

StationTransportSubmitGateDecision PlanStationTransportSubmitGate(
    const StationTransportSubmitGateRequest& request) {
  StationTransportSubmitGateDecision decision;
  decision.mission_id = request.mission_id;

  if (request.mission_already_active_or_queued) {
    decision.message =
        "station transport order already active or queued: " + request.mission_id;
    return decision;
  }
  if (!request.station_catalog_loaded) {
    decision.message = "station transport preflight rejected: failed to load station catalog";
    return decision;
  }
  if (!request.preflight.allowed) {
    decision.message = "station transport preflight rejected: " + request.preflight.message;
    return decision;
  }
  if (!request.route_path.has_value()) {
    decision.message = "station transport preflight rejected: no enabled station route";
    return decision;
  }

  decision.accepted = true;
  decision.route_lock_ids =
      BuildRouteLockIds(*request.route_path, request.traffic_intersection_locks_enabled);
  return decision;
}

const char* StationTransportPhaseName(const StationTransportPhase phase) {
  switch (phase) {
    case StationTransportPhase::kQueued:
      return "QUEUED";
    case StationTransportPhase::kNavigatePickup:
      return "NAV_TO_PICKUP";
    case StationTransportPhase::kWaitLoad:
      return "WAIT_LOAD";
    case StationTransportPhase::kNavigateDropoff:
      return "NAV_TO_DROPOFF";
    case StationTransportPhase::kWaitUnload:
      return "WAIT_UNLOAD";
    case StationTransportPhase::kCompleted:
      return "COMPLETED";
    case StationTransportPhase::kFailed:
      return "FAILED";
    case StationTransportPhase::kCanceled:
      return "CANCELED";
    case StationTransportPhase::kNeedsOperator:
      return "NEEDS_OPERATOR";
  }
  return "UNKNOWN";
}

const char* StationTransportLegName(const StationTransportLeg leg) {
  switch (leg) {
    case StationTransportLeg::kNone:
      return "NONE";
    case StationTransportLeg::kPickup:
      return "PICKUP";
    case StationTransportLeg::kDropoff:
      return "DROPOFF";
  }
  return "UNKNOWN";
}

bool IsStationTransportTerminal(const StationTransportPhase phase) {
  return phase == StationTransportPhase::kCompleted ||
         phase == StationTransportPhase::kFailed ||
         phase == StationTransportPhase::kCanceled;
}

bool IsStationTransportBusy(const StationTransportPhase phase) {
  return !IsStationTransportTerminal(phase);
}

StationTransportTransition PlanStationTransportDispatch(
    const StationTransportRuntimeState& state,
    const StationTransportLeg leg,
    const std::uint64_t dispatch_token) {
  if (!HasRequiredOrderIdentity(state)) {
    return RejectTransition(state, "station transport order identity is incomplete");
  }
  if (dispatch_token == 0 || dispatch_token <= state.dispatch_token) {
    return RejectTransition(state, "station transport dispatch token must advance");
  }

  if (leg == StationTransportLeg::kPickup) {
    if (state.phase != StationTransportPhase::kQueued) {
      return RejectTransition(state, "pickup navigation can only start from QUEUED");
    }
    if (state.payload_loaded || !state.payload_id.empty()) {
      return RejectTransition(state, "pickup navigation requires an empty robot payload");
    }

    auto transition = AcceptTransition(state, "pickup navigation dispatched");
    transition.next_state.phase = StationTransportPhase::kNavigatePickup;
    transition.next_state.active_leg = StationTransportLeg::kPickup;
    transition.next_state.arrived_leg = StationTransportLeg::kNone;
    transition.next_state.dispatch_token = dispatch_token;
    return transition;
  }

  if (leg == StationTransportLeg::kDropoff) {
    if (state.phase != StationTransportPhase::kWaitLoad ||
        state.arrived_leg != StationTransportLeg::kPickup) {
      return RejectTransition(
          state, "dropoff navigation requires confirmed pickup arrival and load");
    }
    if (!state.payload_loaded || state.payload_id.empty()) {
      return RejectTransition(state, "dropoff navigation requires a loaded payload");
    }

    auto transition = AcceptTransition(state, "dropoff navigation dispatched");
    transition.next_state.phase = StationTransportPhase::kNavigateDropoff;
    transition.next_state.active_leg = StationTransportLeg::kDropoff;
    transition.next_state.arrived_leg = StationTransportLeg::kNone;
    transition.next_state.dispatch_token = dispatch_token;
    return transition;
  }

  return RejectTransition(state, "station transport navigation leg is required");
}

StationTransportTransition PlanStationTransportNavigationResult(
    const StationTransportRuntimeState& state,
    const StationTransportLeg leg,
    const std::uint64_t dispatch_token,
    const bool succeeded,
    const bool canceled) {
  if (dispatch_token == 0 || dispatch_token != state.dispatch_token) {
    return RejectTransition(state, "stale station transport navigation result");
  }
  if (succeeded && canceled) {
    return RejectTransition(state, "navigation result cannot succeed and be canceled");
  }

  const auto expected_phase =
      leg == StationTransportLeg::kPickup
          ? StationTransportPhase::kNavigatePickup
          : StationTransportPhase::kNavigateDropoff;
  if (leg == StationTransportLeg::kNone || state.phase != expected_phase ||
      state.active_leg != leg) {
    return RejectTransition(state, "navigation result does not match the active leg");
  }

  if (succeeded) {
    auto transition = AcceptTransition(state, "station navigation completed");
    transition.next_state.phase =
        leg == StationTransportLeg::kPickup ? StationTransportPhase::kWaitLoad
                                           : StationTransportPhase::kWaitUnload;
    transition.next_state.active_leg = StationTransportLeg::kNone;
    transition.next_state.arrived_leg = leg;
    return transition;
  }

  auto transition = AcceptTransition(
      state, canceled ? "station navigation canceled" : "station navigation failed");
  transition.next_state.active_leg = StationTransportLeg::kNone;
  transition.next_state.arrived_leg = StationTransportLeg::kNone;
  if (state.payload_loaded) {
    transition.next_state.phase = StationTransportPhase::kNeedsOperator;
    transition.message += "; loaded payload requires operator handling";
  } else {
    transition.next_state.phase = canceled ? StationTransportPhase::kCanceled
                                           : StationTransportPhase::kFailed;
    transition.release_resources = true;
  }
  return transition;
}

StationTransportTransition PlanStationTransportConfirmLoad(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id) {
  if (state.phase != StationTransportPhase::kWaitLoad ||
      state.arrived_leg != StationTransportLeg::kPickup) {
    return RejectTransition(state, "load confirmation requires pickup arrival");
  }
  if (station_id.empty() || station_id != state.pickup_station_id) {
    return RejectTransition(state, "load confirmation station does not match pickup station");
  }
  if (payload_id.empty()) {
    return RejectTransition(state, "load confirmation payload_id is required");
  }
  if (state.payload_loaded || !state.payload_id.empty()) {
    return RejectTransition(state, "robot payload is already loaded");
  }

  auto transition = AcceptTransition(state, "load confirmed; dropoff navigation required");
  transition.next_state.payload_id = payload_id;
  transition.next_state.payload_loaded = true;
  transition.dispatch_leg = StationTransportLeg::kDropoff;
  return transition;
}

StationTransportTransition PlanStationTransportConfirmUnload(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id) {
  if (state.phase != StationTransportPhase::kWaitUnload ||
      state.arrived_leg != StationTransportLeg::kDropoff) {
    return RejectTransition(state, "unload confirmation requires dropoff arrival");
  }
  if (station_id.empty() || station_id != state.dropoff_station_id) {
    return RejectTransition(state, "unload confirmation station does not match dropoff station");
  }
  if (!state.payload_loaded || state.payload_id.empty()) {
    return RejectTransition(state, "unload confirmation requires a loaded payload");
  }
  if (payload_id.empty() || payload_id != state.payload_id) {
    return RejectTransition(state, "unload confirmation payload does not match loaded payload");
  }

  auto transition = AcceptTransition(state, "unload confirmed; station transport completed");
  transition.next_state.phase = StationTransportPhase::kCompleted;
  transition.next_state.active_leg = StationTransportLeg::kNone;
  transition.next_state.arrived_leg = StationTransportLeg::kNone;
  transition.next_state.payload_id.clear();
  transition.next_state.payload_loaded = false;
  transition.release_resources = true;
  return transition;
}

StationTransportTransition PlanStationTransportCancel(
    const StationTransportRuntimeState& state) {
  if (IsStationTransportTerminal(state.phase)) {
    return RejectTransition(state, "station transport order is already terminal");
  }

  auto transition = AcceptTransition(state, "station transport order canceled");
  transition.next_state.active_leg = StationTransportLeg::kNone;
  transition.next_state.arrived_leg = StationTransportLeg::kNone;
  if (state.payload_loaded) {
    transition.next_state.phase = StationTransportPhase::kNeedsOperator;
    transition.message += "; loaded payload requires operator handling";
  } else {
    transition.next_state.phase = StationTransportPhase::kCanceled;
    transition.release_resources = true;
  }
  return transition;
}

}  // namespace robot_tasks
