#pragma once

#include <string>

namespace robot_tasks {

enum class MissionActionResultCode {
  kSucceeded,
  kCanceled,
  kFailed,
};

struct MissionResultWorkflowInput {
  MissionActionResultCode code = MissionActionResultCode::kFailed;
  bool result_success = false;
  std::string result_message;
  bool cancellation_requested = false;
  bool completed_dock = false;
};

struct MissionResultWorkflowDecision {
  std::string branch;
  bool mission_succeeded = false;
  bool clear_cancellation_requested = false;
  bool mark_docking_terminal = false;
  std::string docking_terminal_state;
  std::string docking_terminal_message;
  bool release_all_resources = false;
  bool release_route_locks = false;
  bool probe_docking_contact_failure = false;
  bool reset_failure_retry_count = false;
  bool mark_resource_occupied = false;
  bool complete_docking = false;
  std::string state;
  std::string state_message;
  bool recoverable = false;
  bool handle_failure = false;
  std::string failure_message;
};

MissionResultWorkflowDecision PlanMissionResultWorkflow(
    const MissionResultWorkflowInput& input);

struct MissionDockAfterResultRequest {
  bool mission_succeeded = false;
  bool return_to_dock_after_mission = false;
  bool completed_dock = false;
  bool mission_queue_empty = true;
  bool dock_profile_available = false;
};

bool ShouldQueueDockAfterResult(const MissionDockAfterResultRequest& request);

struct MissionPostResultQueueRequest {
  bool existing_queue_start_requested = false;
  bool auto_start_queue = true;
  bool queue_running = false;
  bool mission_queue_empty = true;
};

struct MissionPostResultQueueDecision {
  bool queue_start_requested = false;
  bool queue_running = false;
};

MissionPostResultQueueDecision PlanMissionPostResultQueue(
    const MissionPostResultQueueRequest& request);

struct MissionResultPostActionRequest {
  bool mission_succeeded = false;
  bool return_to_dock_after_mission = false;
  bool completed_dock = false;
  bool mission_queue_empty = true;
  bool dock_profile_available = false;
  bool existing_queue_start_requested = false;
  bool auto_start_queue = true;
  bool queue_running = false;
};

struct MissionResultPostActionDecision {
  bool queue_dock_after_result = false;
  MissionPostResultQueueDecision queue;
};

MissionResultPostActionDecision PlanMissionResultPostActions(
    const MissionResultPostActionRequest& request);

}  // namespace robot_tasks
