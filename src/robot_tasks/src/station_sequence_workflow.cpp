#include "robot_tasks/station_sequence_workflow.hpp"

namespace robot_tasks {

StationSequenceAdmissionResult CheckStationSequenceConflicts(
    const std::vector<StationSequenceLeg>& legs,
    const std::unordered_set<std::string>& active_or_queued_mission_ids) {
  for (const auto& leg : legs) {
    if (active_or_queued_mission_ids.count(leg.profile.mission_id) > 0U) {
      return StationSequenceAdmissionResult{
          false, "station sequence leg already active or queued: " + leg.profile.mission_id};
    }
  }
  return StationSequenceAdmissionResult{};
}

StationSequenceAdmissionResult CheckStationSequenceLegPreflight(
    const StationSequenceLeg& leg, const MissionPreflightResult& preflight) {
  if (preflight.allowed) {
    return StationSequenceAdmissionResult{};
  }
  return StationSequenceAdmissionResult{
      false,
      "station sequence preflight rejected for " + leg.profile.mission_id + ": " +
          preflight.message};
}

std::vector<std::string> StationSequenceMissionIds(
    const std::vector<StationSequenceLeg>& legs) {
  std::vector<std::string> mission_ids;
  mission_ids.reserve(legs.size());
  for (const auto& leg : legs) {
    mission_ids.push_back(leg.profile.mission_id);
  }
  return mission_ids;
}

std::vector<QueuedMission> BuildStationSequenceQueuedMissions(
    const std::vector<StationSequenceLeg>& legs, const int priority,
    const std::uint64_t first_sequence) {
  std::vector<QueuedMission> queued_missions;
  queued_missions.reserve(legs.size());
  for (std::size_t index = 0; index < legs.size(); ++index) {
    const auto& leg = legs[index];
    queued_missions.push_back(QueuedMission{
        leg.profile, "station_sequence:" + leg.profile.mission_id, priority,
        first_sequence + index});
  }
  return queued_missions;
}

std::string BuildStationSequenceQueuedMessage(const std::size_t queued_count) {
  return "queued station sequence with " + std::to_string(queued_count) + " leg(s)";
}

}  // namespace robot_tasks
