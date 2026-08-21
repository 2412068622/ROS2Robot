#include "robot_tasks/mission_event_log.hpp"

#include <algorithm>
#include <utility>

namespace robot_tasks {

MissionEvent MakeMissionEvent(
    const std::string& stamp, const std::string& mission_id, const std::string& state,
    const std::string& previous_state, const std::string& message, const bool recoverable) {
  MissionEvent event;
  event.stamp = stamp;
  event.mission_id = mission_id;
  event.state = state;
  event.previous_state = previous_state;
  event.message = message;
  event.recoverable = recoverable;
  return event;
}

std::size_t NormalizeEventLimit(
    const int requested_limit, const std::size_t default_limit, const std::size_t max_limit) {
  const auto effective_limit =
      requested_limit > 0 ? static_cast<std::size_t>(requested_limit) : default_limit;
  return std::min(effective_limit, max_limit);
}

std::size_t TrimMissionEvents(std::vector<MissionEvent>& events, const std::size_t max_events) {
  if (events.size() <= max_events) {
    return 0U;
  }
  const auto removed = events.size() - max_events;
  events.erase(events.begin(), events.begin() + static_cast<std::ptrdiff_t>(removed));
  return removed;
}

void AppendMissionEvent(
    std::vector<MissionEvent>& events, MissionEvent event, const std::size_t max_events) {
  events.push_back(std::move(event));
  TrimMissionEvents(events, max_events);
}

std::vector<MissionEvent> SelectRecentMissionEvents(
    const std::vector<MissionEvent>& events, const std::size_t limit,
    const std::string& state_filter, const std::string& mission_id_filter) {
  std::vector<MissionEvent> selected;
  selected.reserve(std::min(limit, events.size()));
  for (auto it = events.rbegin(); it != events.rend() && selected.size() < limit; ++it) {
    if (!state_filter.empty() && it->state != state_filter) {
      continue;
    }
    if (!mission_id_filter.empty() && it->mission_id != mission_id_filter) {
      continue;
    }
    selected.push_back(*it);
  }
  return selected;
}

}  // namespace robot_tasks
