#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/mission_preflight.hpp"
#include "robot_tasks/mission_profile_builders.hpp"
#include "robot_tasks/mission_queue.hpp"

namespace robot_tasks {

struct StationSequenceAdmissionResult {
  bool accepted = true;
  std::string message = "station sequence accepted";
};

StationSequenceAdmissionResult CheckStationSequenceConflicts(
    const std::vector<StationSequenceLeg>& legs,
    const std::unordered_set<std::string>& active_or_queued_mission_ids);

StationSequenceAdmissionResult CheckStationSequenceLegPreflight(
    const StationSequenceLeg& leg, const MissionPreflightResult& preflight);

std::vector<std::string> StationSequenceMissionIds(
    const std::vector<StationSequenceLeg>& legs);

std::vector<QueuedMission> BuildStationSequenceQueuedMissions(
    const std::vector<StationSequenceLeg>& legs, int priority,
    std::uint64_t first_sequence);

std::string BuildStationSequenceQueuedMessage(std::size_t queued_count);

}  // namespace robot_tasks
