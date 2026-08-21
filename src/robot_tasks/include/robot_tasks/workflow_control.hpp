#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/mission_event_log.hpp"
#include "robot_tasks/mission_queue.hpp"

namespace robot_tasks {

struct WorkflowActiveMission {
  bool active = false;
  bool paused = false;
  std::string mission_id;
};

struct WorkflowCancelRequest {
  std::string order_id;
  bool cancel_active = false;
  std::vector<QueuedMission> queue;
  WorkflowActiveMission active;
};

struct WorkflowCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  std::string message;
  std::vector<std::string> canceled_mission_ids;
  std::vector<QueuedMission> remaining_queue;
};

struct WorkflowPauseRequest {
  std::string order_id;
  bool pause_active = false;
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
  WorkflowActiveMission active;
};

struct WorkflowPauseDecision {
  bool success = false;
  bool active_pause_requested = false;
  bool runner_paused = false;
  bool clear_queue_start_requested = false;
  std::string message;
  std::vector<std::string> affected_mission_ids;
  std::unordered_set<std::string> paused_order_ids;
};

struct WorkflowResumeRequest {
  std::string order_id;
  bool resume_active = false;
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
  WorkflowActiveMission active;
};

struct WorkflowResumeDecision {
  bool success = false;
  bool active_resume_requested = false;
  bool runner_paused = false;
  bool queue_running = false;
  bool queue_start_requested = false;
  std::string message;
  std::vector<std::string> affected_mission_ids;
  std::unordered_set<std::string> paused_order_ids;
};

struct WorkflowStatusRequest {
  std::string order_id;
  std::vector<QueuedMission> queue;
  WorkflowActiveMission active;
  std::unordered_set<std::string> paused_order_ids;
  std::vector<MissionEvent> events;
  std::string current_state;
};

struct WorkflowStatusSnapshot {
  bool success = false;
  bool paused = false;
  std::string message;
  std::string state;
  std::string running_mission_id;
  std::vector<std::string> queued_mission_ids;
  std::vector<std::string> finished_mission_ids;
  int total_steps = 0;
  int finished_steps = 0;
  int queued_steps = 0;
};

WorkflowCancelDecision PlanWorkflowCancel(const WorkflowCancelRequest& request);
WorkflowPauseDecision PlanWorkflowPause(const WorkflowPauseRequest& request);
WorkflowResumeDecision PlanWorkflowResume(const WorkflowResumeRequest& request);
WorkflowStatusSnapshot BuildWorkflowStatusSnapshot(const WorkflowStatusRequest& request);

}  // namespace robot_tasks
