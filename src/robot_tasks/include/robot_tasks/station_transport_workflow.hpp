#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/mission_preflight.hpp"

namespace robot_tasks {

struct StationTransportSubmitGateRequest {
  std::string mission_id;
  bool mission_already_active_or_queued = false;
  bool station_catalog_loaded = true;
  MissionPreflightResult preflight;
  std::optional<std::vector<std::string>> route_path;
  bool traffic_intersection_locks_enabled = false;
};

struct StationTransportSubmitGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string message;
  std::vector<std::string> route_lock_ids;
};

StationTransportSubmitGateDecision PlanStationTransportSubmitGate(
    const StationTransportSubmitGateRequest& request);

enum class StationTransportPhase {
  kQueued,
  kNavigatePickup,
  kWaitLoad,
  kNavigateDropoff,
  kWaitUnload,
  kCompleted,
  kFailed,
  kCanceled,
  kNeedsOperator,
};

enum class StationTransportLeg {
  kNone,
  kPickup,
  kDropoff,
};

struct StationTransportRuntimeState {
  std::string mission_id;
  std::string pickup_station_id;
  std::string dropoff_station_id;
  std::string payload_id;
  StationTransportPhase phase = StationTransportPhase::kQueued;
  StationTransportLeg active_leg = StationTransportLeg::kNone;
  StationTransportLeg arrived_leg = StationTransportLeg::kNone;
  std::uint64_t dispatch_token = 0;
  bool payload_loaded = false;
};

struct StationTransportTransition {
  bool accepted = false;
  StationTransportRuntimeState next_state;
  std::string message;
  std::optional<StationTransportLeg> dispatch_leg;
  bool release_resources = false;
};

const char* StationTransportPhaseName(StationTransportPhase phase);
const char* StationTransportLegName(StationTransportLeg leg);
bool IsStationTransportTerminal(StationTransportPhase phase);
bool IsStationTransportBusy(StationTransportPhase phase);

StationTransportTransition PlanStationTransportDispatch(
    const StationTransportRuntimeState& state,
    StationTransportLeg leg,
    std::uint64_t dispatch_token);

StationTransportTransition PlanStationTransportNavigationResult(
    const StationTransportRuntimeState& state,
    StationTransportLeg leg,
    std::uint64_t dispatch_token,
    bool succeeded,
    bool canceled);

StationTransportTransition PlanStationTransportConfirmLoad(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id);

StationTransportTransition PlanStationTransportConfirmUnload(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id);

StationTransportTransition PlanStationTransportCancel(
    const StationTransportRuntimeState& state);

}  // namespace robot_tasks
