#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_tasks {

struct MissionEvent {
  std::string stamp;
  std::string mission_id;
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = true;
};

MissionEvent MakeMissionEvent(
    const std::string& stamp, const std::string& mission_id, const std::string& state,
    const std::string& previous_state, const std::string& message, bool recoverable);

std::size_t NormalizeEventLimit(
    int requested_limit, std::size_t default_limit, std::size_t max_limit);

std::size_t TrimMissionEvents(std::vector<MissionEvent>& events, std::size_t max_events);

void AppendMissionEvent(
    std::vector<MissionEvent>& events, MissionEvent event, std::size_t max_events);

std::vector<MissionEvent> SelectRecentMissionEvents(
    const std::vector<MissionEvent>& events, std::size_t limit,
    const std::string& state_filter = "", const std::string& mission_id_filter = "");

}  // namespace robot_tasks
