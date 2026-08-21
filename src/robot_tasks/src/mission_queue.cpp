#include "robot_tasks/mission_queue.hpp"

#include <algorithm>

namespace robot_tasks {
namespace {

int QueueSizeAsInt(const std::size_t queue_size) {
  return static_cast<int>(queue_size);
}

}  // namespace

void SortMissionQueue(std::vector<QueuedMission>& queue) {
  std::stable_sort(queue.begin(), queue.end(), [](const QueuedMission& lhs, const QueuedMission& rhs) {
    if (lhs.priority != rhs.priority) {
      return lhs.priority > rhs.priority;
    }
    return lhs.sequence < rhs.sequence;
  });
}

bool MissionIdQueued(const std::vector<QueuedMission>& queue, const std::string& mission_id) {
  return std::any_of(queue.begin(), queue.end(), [&](const QueuedMission& mission) {
    return mission.profile.mission_id == mission_id;
  });
}

std::size_t RemoveQueuedMissionById(
    std::vector<QueuedMission>& queue, const std::string& mission_id) {
  const auto before = queue.size();
  queue.erase(
      std::remove_if(
          queue.begin(), queue.end(),
          [&](const QueuedMission& mission) { return mission.profile.mission_id == mission_id; }),
      queue.end());
  return before - queue.size();
}

bool ReprioritizeQueuedMission(
    std::vector<QueuedMission>& queue, const std::string& mission_id, const int priority) {
  for (auto& mission : queue) {
    if (mission.profile.mission_id == mission_id) {
      mission.priority = priority;
      return true;
    }
  }
  return false;
}

QueuedMissionStartDecision DecideQueuedMissionStart(
    const QueuedMissionStartRequest& request) {
  QueuedMissionStartDecision decision;
  if (request.preempt_current && request.mission_active) {
    decision.run_queue = true;
    decision.request_queue_start = true;
    decision.cancel_active = true;
    return decision;
  }
  if (request.start_if_idle && !request.mission_active) {
    decision.run_queue = true;
    decision.request_queue_start = true;
  }
  return decision;
}

bool MissionBelongsToWorkflowOrder(
    const std::string& mission_id, const std::string& order_id) {
  const auto fleet_prefix = "station_order_fleet_" + order_id + "_step_";
  const auto facility_prefix = "facility_action_" + order_id + "_step_";
  const auto station_sequence_prefix = "station_sequence_" + order_id + "_step_";
  return mission_id.rfind(fleet_prefix, 0) == 0 ||
         mission_id.rfind(facility_prefix, 0) == 0 ||
         mission_id.rfind(station_sequence_prefix, 0) == 0;
}

bool MissionBelongsToStationOrderBatch(
    const std::string& mission_id, const std::string& batch_id) {
  const auto batch_prefix = "station_order_batch_" + batch_id + "_order_";
  return mission_id.rfind(batch_prefix, 0) == 0;
}

std::optional<std::string> PausedWorkflowForMission(
    const std::string& mission_id, const std::unordered_set<std::string>& paused_order_ids) {
  for (const auto& order_id : paused_order_ids) {
    if (MissionBelongsToWorkflowOrder(mission_id, order_id)) {
      return order_id;
    }
  }
  return std::nullopt;
}

std::vector<std::string> QueuedWorkflowMissionIds(
    const std::vector<QueuedMission>& queue, const std::string& order_id) {
  std::vector<std::string> mission_ids;
  for (const auto& mission : queue) {
    if (MissionBelongsToWorkflowOrder(mission.profile.mission_id, order_id)) {
      mission_ids.push_back(mission.profile.mission_id);
    }
  }
  return mission_ids;
}

std::size_t RemoveQueuedWorkflowMissions(
    std::vector<QueuedMission>& queue, const std::string& order_id,
    std::vector<std::string>* removed_mission_ids) {
  const auto before = queue.size();
  const auto removed_end = std::remove_if(
      queue.begin(), queue.end(), [&](const QueuedMission& mission) {
        if (!MissionBelongsToWorkflowOrder(mission.profile.mission_id, order_id)) {
          return false;
        }
        if (removed_mission_ids != nullptr) {
          removed_mission_ids->push_back(mission.profile.mission_id);
        }
        return true;
      });
  queue.erase(removed_end, queue.end());
  return before - queue.size();
}

std::size_t RemoveQueuedStationOrderBatchMissions(
    std::vector<QueuedMission>& queue, const std::string& batch_id,
    std::vector<std::string>* removed_mission_ids) {
  const auto before = queue.size();
  const auto removed_end = std::remove_if(
      queue.begin(), queue.end(), [&](const QueuedMission& mission) {
        if (!MissionBelongsToStationOrderBatch(mission.profile.mission_id, batch_id)) {
          return false;
        }
        if (removed_mission_ids != nullptr) {
          removed_mission_ids->push_back(mission.profile.mission_id);
        }
        return true;
      });
  queue.erase(removed_end, queue.end());
  return before - queue.size();
}

std::vector<std::string> ReprioritizeQueuedMissionBatch(
    std::vector<QueuedMission>& queue, const std::vector<std::string>& mission_ids,
    const std::vector<int>& priorities) {
  std::vector<std::string> updated_mission_ids;
  const auto count = std::min(mission_ids.size(), priorities.size());
  for (std::size_t index = 0; index < count; ++index) {
    for (auto& mission : queue) {
      if (mission.profile.mission_id == mission_ids[index]) {
        mission.priority = priorities[index];
        updated_mission_ids.push_back(mission_ids[index]);
        break;
      }
    }
  }
  return updated_mission_ids;
}

std::optional<std::size_t> FindNextRunnableMissionIndex(
    const std::vector<QueuedMission>& queue,
    const std::unordered_set<std::string>& paused_order_ids) {
  for (std::size_t index = 0; index < queue.size(); ++index) {
    if (!PausedWorkflowForMission(queue[index].profile.mission_id, paused_order_ids).has_value()) {
      return index;
    }
  }
  return std::nullopt;
}

MissionQueueDispatchResult PopNextRunnableMission(
    std::vector<QueuedMission> queue,
    const std::unordered_set<std::string>& paused_order_ids) {
  MissionQueueDispatchResult result;
  result.remaining_queue = queue;
  if (queue.empty()) {
    result.queue_empty = true;
    result.message = "mission queue is empty";
    return result;
  }

  const auto next_index = FindNextRunnableMissionIndex(queue, paused_order_ids);
  if (!next_index.has_value()) {
    result.all_paused = true;
    result.message = "all queued workflow missions are paused";
    return result;
  }

  auto next_it = result.remaining_queue.begin() + static_cast<std::ptrdiff_t>(*next_index);
  result.mission = *next_it;
  result.remaining_queue.erase(next_it);
  result.success = true;
  result.message = "queued mission selected";
  return result;
}

MissionQueueAdmissionDecision PlanMissionQueueAdmission(
    const MissionQueueAdmissionRequest& request) {
  MissionQueueAdmissionDecision decision;
  decision.queue_size = QueueSizeAsInt(request.queue_size);
  decision.message = request.message;
  decision.set_state = request.set_state_when_idle && !request.mission_active;
  decision.state_message = request.message;
  return decision;
}

MissionSubmitGateDecision PlanMissionSubmitGate(
    const MissionSubmitGateRequest& request) {
  MissionSubmitGateDecision decision;
  decision.mission_id = request.mission_id;

  if (request.mission_already_active_or_queued) {
    decision.reject_reason = "duplicate";
    decision.message = request.duplicate_message_prefix +
                       " already active or queued: " + request.mission_id;
    return decision;
  }

  if (!request.preflight.allowed) {
    decision.reject_reason = "preflight";
    decision.message = request.preflight_message_prefix +
                       " preflight rejected: " + request.preflight.message;
    return decision;
  }

  decision.accepted = true;
  decision.reject_reason = "none";
  return decision;
}

QueuedMissionCancelDecision PlanQueuedMissionCancel(
    const QueuedMissionCancelRequest& request) {
  QueuedMissionCancelDecision decision;
  decision.remaining_queue = request.queue;

  const auto removed = RemoveQueuedMissionById(decision.remaining_queue, request.mission_id);
  if (removed > 0U) {
    decision.success = true;
    decision.release_mission_ids.push_back(request.mission_id);
    decision.message = "removed queued mission " + request.mission_id;
    decision.set_state = !request.mission_active;
    decision.state_message = decision.message;
    decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
    return decision;
  }

  if (request.cancel_active && request.mission_active &&
      request.active_mission_id == request.mission_id) {
    decision.success = true;
    decision.active_cancel_requested = true;
    decision.message = "active mission cancel requested: " + request.mission_id;
    decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
    return decision;
  }

  decision.message = "mission not queued" +
                     std::string(request.cancel_active ? " or active: " : ": ") +
                     request.mission_id;
  decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
  return decision;
}

StationOrderBatchCancelDecision PlanStationOrderBatchCancel(
    const StationOrderBatchCancelRequest& request) {
  StationOrderBatchCancelDecision decision;
  decision.remaining_queue = request.queue;

  RemoveQueuedStationOrderBatchMissions(
      decision.remaining_queue, request.batch_id, &decision.canceled_mission_ids);
  decision.release_mission_ids = decision.canceled_mission_ids;

  if (request.cancel_active && request.mission_active &&
      MissionBelongsToStationOrderBatch(request.active_mission_id, request.batch_id)) {
    decision.active_cancel_requested = true;
    decision.canceled_mission_ids.push_back(request.active_mission_id);
  }

  decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
  decision.success = !decision.canceled_mission_ids.empty() ||
                     decision.active_cancel_requested;
  decision.message =
      decision.success
          ? "canceled station order batch " + request.batch_id + " mission_count=" +
                std::to_string(decision.canceled_mission_ids.size())
          : "station order batch has no queued" +
                std::string(request.cancel_active ? " or active" : "") +
                " mission: " + request.batch_id;
  decision.set_state = decision.success && !request.mission_active;
  decision.state_message = decision.message;
  return decision;
}

StationOrderBatchSubmitPlan PlanStationOrderBatchSubmit(
    const StationOrderBatchSubmitRequest& request) {
  StationOrderBatchSubmitPlan plan;
  if (request.batch_id.empty()) {
    plan.message = "batch_id is empty";
    return plan;
  }

  const auto order_count = request.original_order_ids.size();
  if (order_count == 0U || request.sanitized_order_ids.size() != order_count ||
      request.pickup_station_ids.size() != order_count ||
      request.dropoff_station_ids.size() != order_count) {
    plan.message = "batch order_ids, pickup_station_ids and dropoff_station_ids size mismatch";
    return plan;
  }

  plan.accepted = true;
  for (std::size_t index = 0; index < order_count; ++index) {
    StationOrderBatchItemPlan item;
    item.original_order_id = request.original_order_ids[index];
    item.order_id = request.sanitized_order_ids[index];
    if (item.order_id.empty()) {
      item.reject_order_id = item.original_order_id;
      item.reject_message = "order_id is empty";
      plan.items.push_back(item);
      if (!request.continue_on_error) {
        break;
      }
      continue;
    }

    item.accepted = true;
    item.station_order_id =
        "batch_" + request.batch_id + "_order_" + item.order_id;
    item.pickup_station = request.pickup_station_ids[index];
    item.dropoff_station = request.dropoff_station_ids[index];
    item.priority = request.priority - static_cast<int>(index);
    item.start_if_idle = request.start_if_idle && index == 0U;
    item.preempt_current = request.preempt_current && index == 0U;
    plan.items.push_back(item);
  }
  return plan;
}

StationOrderBatchSubmitSummary PlanStationOrderBatchSubmitSummary(
    const std::string& batch_id, const bool continue_on_error, const int accepted_count,
    const int rejected_count) {
  StationOrderBatchSubmitSummary summary;
  summary.accepted_count = accepted_count;
  summary.rejected_count = rejected_count;
  summary.success = accepted_count > 0 && rejected_count == 0;
  if (continue_on_error && accepted_count > 0) {
    summary.success = true;
  }
  summary.message = "station order batch " + batch_id + " accepted=" +
                    std::to_string(accepted_count) + " rejected=" +
                    std::to_string(rejected_count);
  return summary;
}

QueuedMissionReprioritizeDecision PlanQueuedMissionReprioritize(
    const QueuedMissionReprioritizeRequest& request) {
  QueuedMissionReprioritizeDecision decision;
  decision.remaining_queue = request.queue;
  if (ReprioritizeQueuedMission(decision.remaining_queue, request.mission_id, request.priority)) {
    SortMissionQueue(decision.remaining_queue);
    decision.success = true;
    decision.updated_mission_ids.push_back(request.mission_id);
    decision.message = "reprioritized queued mission " + request.mission_id + " to " +
                       std::to_string(request.priority);
    decision.set_state = !request.mission_active;
    decision.state_message = decision.message;
    decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
    return decision;
  }

  decision.message = "mission not queued: " + request.mission_id;
  decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
  return decision;
}

QueuedMissionReprioritizeDecision PlanQueuedMissionBatchReprioritize(
    const QueuedMissionBatchReprioritizeRequest& request) {
  QueuedMissionReprioritizeDecision decision;
  decision.remaining_queue = request.queue;
  if (request.mission_ids.size() != request.priorities.size()) {
    decision.message = "mission_ids and priorities size mismatch";
    decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
    return decision;
  }

  decision.updated_mission_ids = ReprioritizeQueuedMissionBatch(
      decision.remaining_queue, request.mission_ids, request.priorities);
  SortMissionQueue(decision.remaining_queue);
  decision.success = decision.updated_mission_ids.size() == request.mission_ids.size();
  decision.message = "reprioritized " +
                     std::to_string(decision.updated_mission_ids.size()) +
                     " queued mission(s)";
  decision.set_state = !request.mission_active && !decision.updated_mission_ids.empty();
  decision.state_message = decision.message;
  decision.queue_size = QueueSizeAsInt(decision.remaining_queue.size());
  return decision;
}

}  // namespace robot_tasks
