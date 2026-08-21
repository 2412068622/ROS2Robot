#include "robot_tasks/mission_result_workflow.hpp"

namespace robot_tasks {
namespace {

std::string ResultMessageOr(const std::string& message, const std::string& fallback) {
  return message.empty() ? fallback : message;
}

}  // namespace

MissionResultWorkflowDecision PlanMissionResultWorkflow(
    const MissionResultWorkflowInput& input) {
  MissionResultWorkflowDecision decision;

  if (input.code == MissionActionResultCode::kCanceled || input.cancellation_requested) {
    decision.branch = "canceled";
    decision.clear_cancellation_requested = true;
    decision.mark_docking_terminal = true;
    decision.docking_terminal_state = "CANCELED";
    decision.docking_terminal_message = "docking mission canceled";
    decision.release_all_resources = true;
    decision.state = "CANCELED";
    decision.state_message = ResultMessageOr(input.result_message, "mission canceled");
    decision.recoverable = true;
    return decision;
  }

  if (input.code == MissionActionResultCode::kSucceeded && input.result_success) {
    decision.branch = "succeeded";
    decision.mission_succeeded = true;
    decision.release_all_resources = !input.completed_dock;
    decision.probe_docking_contact_failure = true;
    decision.reset_failure_retry_count = true;
    decision.mark_resource_occupied = true;
    decision.complete_docking = true;
    decision.state = "FINISHED";
    decision.state_message = ResultMessageOr(input.result_message, "mission finished");
    decision.recoverable = true;
    return decision;
  }

  decision.branch = "failed";
  decision.mark_docking_terminal = true;
  decision.docking_terminal_state = "FAILED";
  decision.docking_terminal_message = ResultMessageOr(input.result_message, "mission failed");
  decision.release_all_resources = true;
  decision.handle_failure = true;
  decision.failure_message = decision.docking_terminal_message;
  return decision;
}

bool ShouldQueueDockAfterResult(const MissionDockAfterResultRequest& request) {
  return request.mission_succeeded && request.return_to_dock_after_mission &&
         !request.completed_dock && request.mission_queue_empty &&
         request.dock_profile_available;
}

MissionPostResultQueueDecision PlanMissionPostResultQueue(
    const MissionPostResultQueueRequest& request) {
  MissionPostResultQueueDecision decision;
  decision.queue_start_requested =
      !request.mission_queue_empty &&
      (request.existing_queue_start_requested ||
       (request.auto_start_queue && request.queue_running && !request.mission_queue_empty));
  decision.queue_running = request.queue_running;
  if (request.mission_queue_empty) {
    decision.queue_running = false;
  }
  return decision;
}

MissionResultPostActionDecision PlanMissionResultPostActions(
    const MissionResultPostActionRequest& request) {
  MissionResultPostActionDecision decision;
  decision.queue_dock_after_result = ShouldQueueDockAfterResult(
      MissionDockAfterResultRequest{
          request.mission_succeeded, request.return_to_dock_after_mission,
          request.completed_dock, request.mission_queue_empty,
          request.dock_profile_available});

  const bool queue_start_requested =
      request.existing_queue_start_requested || decision.queue_dock_after_result;
  const bool queue_running = request.queue_running || decision.queue_dock_after_result;
  const bool queue_empty = request.mission_queue_empty && !decision.queue_dock_after_result;
  decision.queue = PlanMissionPostResultQueue(
      MissionPostResultQueueRequest{
          queue_start_requested, request.auto_start_queue, queue_running, queue_empty});
  return decision;
}

}  // namespace robot_tasks
