#pragma once

#include <string>

namespace robot_tasks {

struct MissionRuntimeTickInput {
  bool autostart = false;
  bool autostart_sent = false;
  bool mission_active = false;
  bool queue_start_requested = false;
};

struct MissionRuntimeTickDecision {
  bool start_default_mission = false;
  bool start_queued_mission = false;
  bool clear_queue_start_request = false;
  std::string branch = "publish_only";
};

MissionRuntimeTickDecision DecideMissionRuntimeTick(const MissionRuntimeTickInput& input);

}  // namespace robot_tasks
