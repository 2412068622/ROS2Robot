#include "robot_tasks/workflow_control.hpp"

#include <algorithm>
#include <unordered_set>

namespace robot_tasks {
namespace {

bool ActiveMissionBelongsToWorkflow(
    const WorkflowActiveMission& active, const std::string& order_id) {
  return active.active && MissionBelongsToWorkflowOrder(active.mission_id, order_id);
}

}  // namespace

WorkflowCancelDecision PlanWorkflowCancel(const WorkflowCancelRequest& request) {
  WorkflowCancelDecision decision;
  decision.remaining_queue = request.queue;

  RemoveQueuedWorkflowMissions(
      decision.remaining_queue, request.order_id, &decision.canceled_mission_ids);

  if (request.cancel_active && ActiveMissionBelongsToWorkflow(request.active, request.order_id)) {
    decision.active_cancel_requested = true;
    decision.canceled_mission_ids.push_back(request.active.mission_id);
  }

  decision.success = !decision.canceled_mission_ids.empty() ||
                     decision.active_cancel_requested;
  decision.message =
      decision.success
          ? "canceled workflow " + request.order_id + " mission_count=" +
                std::to_string(decision.canceled_mission_ids.size())
          : "workflow has no queued" +
                std::string(request.cancel_active ? " or active" : "") +
                " mission: " + request.order_id;
  return decision;
}

WorkflowPauseDecision PlanWorkflowPause(const WorkflowPauseRequest& request) {
  WorkflowPauseDecision decision;
  decision.paused_order_ids = request.paused_order_ids;
  decision.affected_mission_ids = QueuedWorkflowMissionIds(request.queue, request.order_id);

  if (request.pause_active && ActiveMissionBelongsToWorkflow(request.active, request.order_id)) {
    decision.active_pause_requested = true;
    decision.runner_paused = true;
    decision.affected_mission_ids.push_back(request.active.mission_id);
  }

  if (!decision.affected_mission_ids.empty()) {
    decision.paused_order_ids.insert(request.order_id);
    decision.clear_queue_start_requested = true;
    decision.success = true;
    decision.message = "paused workflow " + request.order_id + " mission_count=" +
                       std::to_string(decision.affected_mission_ids.size());
  } else {
    decision.success = false;
    decision.message = "workflow has no queued" +
                       std::string(request.pause_active ? " or active" : "") +
                       " mission: " + request.order_id;
  }

  return decision;
}

WorkflowResumeDecision PlanWorkflowResume(const WorkflowResumeRequest& request) {
  WorkflowResumeDecision decision;
  decision.paused_order_ids = request.paused_order_ids;
  decision.affected_mission_ids = QueuedWorkflowMissionIds(request.queue, request.order_id);

  if (request.resume_active && request.active.paused &&
      ActiveMissionBelongsToWorkflow(request.active, request.order_id)) {
    decision.active_resume_requested = true;
    decision.runner_paused = false;
    decision.affected_mission_ids.push_back(request.active.mission_id);
  }

  const bool was_paused = decision.paused_order_ids.erase(request.order_id) > 0U;
  if (was_paused || decision.active_resume_requested ||
      !decision.affected_mission_ids.empty()) {
    if (!request.active.active && !decision.affected_mission_ids.empty()) {
      decision.queue_running = true;
      decision.queue_start_requested = true;
    }
    decision.success = true;
    decision.message = "resumed workflow " + request.order_id + " mission_count=" +
                       std::to_string(decision.affected_mission_ids.size());
  } else {
    decision.success = false;
    decision.message = "workflow is not paused or queued: " + request.order_id;
  }

  return decision;
}

WorkflowStatusSnapshot BuildWorkflowStatusSnapshot(const WorkflowStatusRequest& request) {
  WorkflowStatusSnapshot snapshot;
  std::unordered_set<std::string> all_steps;
  std::unordered_set<std::string> finished_steps;

  for (const auto& mission : request.queue) {
    if (MissionBelongsToWorkflowOrder(mission.profile.mission_id, request.order_id)) {
      snapshot.queued_mission_ids.push_back(mission.profile.mission_id);
      all_steps.insert(mission.profile.mission_id);
    }
  }

  if (ActiveMissionBelongsToWorkflow(request.active, request.order_id)) {
    snapshot.running_mission_id = request.active.mission_id;
    all_steps.insert(request.active.mission_id);
  }

  for (const auto& event : request.events) {
    if (event.state == "FINISHED" &&
        MissionBelongsToWorkflowOrder(event.mission_id, request.order_id)) {
      finished_steps.insert(event.mission_id);
      all_steps.insert(event.mission_id);
    }
  }

  snapshot.finished_mission_ids.assign(finished_steps.begin(), finished_steps.end());
  std::sort(snapshot.finished_mission_ids.begin(), snapshot.finished_mission_ids.end());
  snapshot.paused = request.paused_order_ids.find(request.order_id) !=
                    request.paused_order_ids.end();
  snapshot.total_steps = static_cast<int>(all_steps.size());
  snapshot.finished_steps = static_cast<int>(finished_steps.size());
  snapshot.queued_steps = static_cast<int>(snapshot.queued_mission_ids.size());

  if (snapshot.paused) {
    snapshot.state = "PAUSED";
  } else if (!snapshot.running_mission_id.empty()) {
    snapshot.state = request.current_state;
  } else if (snapshot.total_steps > 0 && snapshot.finished_steps == snapshot.total_steps) {
    snapshot.state = "FINISHED";
  } else if (snapshot.queued_steps > 0) {
    snapshot.state = "QUEUED";
  } else {
    snapshot.state = "UNKNOWN";
  }

  snapshot.success = snapshot.total_steps > 0;
  snapshot.message = snapshot.success
                         ? "workflow " + request.order_id + " state=" + snapshot.state
                         : "workflow has no known mission: " + request.order_id;
  return snapshot;
}

}  // namespace robot_tasks
