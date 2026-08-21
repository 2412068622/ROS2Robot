#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/mission_preflight.hpp"
#include "robot_tasks/mission_profile.hpp"

namespace robot_tasks {

struct QueuedMission {
  MissionProfile profile;
  std::string mission_file;
  int priority = 0;
  std::uint64_t sequence = 0;
};

struct QueuedMissionStartRequest {
  bool start_if_idle = false;
  bool preempt_current = false;
  bool mission_active = false;
};

struct QueuedMissionStartDecision {
  bool run_queue = false;
  bool request_queue_start = false;
  bool cancel_active = false;
};

struct MissionQueueDispatchResult {
  bool success = false;
  bool queue_empty = false;
  bool all_paused = false;
  std::string message;
  std::optional<QueuedMission> mission;
  std::vector<QueuedMission> remaining_queue;
};

struct MissionQueueAdmissionRequest {
  bool mission_active = false;
  std::size_t queue_size = 0;
  std::string message;
  bool set_state_when_idle = true;
};

struct MissionQueueAdmissionDecision {
  bool success = true;
  int queue_size = 0;
  std::string message;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

struct MissionSubmitGateRequest {
  std::string mission_id;
  std::string duplicate_message_prefix = "mission";
  std::string preflight_message_prefix = "mission";
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

struct MissionSubmitGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string reject_reason;
  std::string message;
};

struct QueuedMissionCancelRequest {
  std::vector<QueuedMission> queue;
  std::string mission_id;
  bool cancel_active = false;
  bool mission_active = false;
  std::string active_mission_id;
};

struct QueuedMissionCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> release_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

struct StationOrderBatchCancelRequest {
  std::vector<QueuedMission> queue;
  std::string batch_id;
  bool cancel_active = false;
  bool mission_active = false;
  std::string active_mission_id;
};

struct StationOrderBatchCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> canceled_mission_ids;
  std::vector<std::string> release_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

struct StationOrderBatchSubmitRequest {
  std::string batch_id;
  std::vector<std::string> original_order_ids;
  std::vector<std::string> sanitized_order_ids;
  std::vector<std::string> pickup_station_ids;
  std::vector<std::string> dropoff_station_ids;
  int priority = 0;
  bool start_if_idle = false;
  bool preempt_current = false;
  bool continue_on_error = false;
};

struct StationOrderBatchItemPlan {
  bool accepted = false;
  std::string original_order_id;
  std::string order_id;
  std::string station_order_id;
  std::string pickup_station;
  std::string dropoff_station;
  int priority = 0;
  bool start_if_idle = false;
  bool preempt_current = false;
  std::string reject_order_id;
  std::string reject_message;
};

struct StationOrderBatchSubmitPlan {
  bool accepted = false;
  std::string message;
  std::vector<StationOrderBatchItemPlan> items;
};

struct StationOrderBatchSubmitSummary {
  bool success = false;
  int accepted_count = 0;
  int rejected_count = 0;
  std::string message;
};

struct QueuedMissionReprioritizeRequest {
  std::vector<QueuedMission> queue;
  std::string mission_id;
  int priority = 0;
  bool mission_active = false;
};

struct QueuedMissionBatchReprioritizeRequest {
  std::vector<QueuedMission> queue;
  std::vector<std::string> mission_ids;
  std::vector<int> priorities;
  bool mission_active = false;
};

struct QueuedMissionReprioritizeDecision {
  bool success = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> updated_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

void SortMissionQueue(std::vector<QueuedMission>& queue);
bool MissionIdQueued(const std::vector<QueuedMission>& queue, const std::string& mission_id);
std::size_t RemoveQueuedMissionById(
    std::vector<QueuedMission>& queue, const std::string& mission_id);
bool ReprioritizeQueuedMission(
    std::vector<QueuedMission>& queue, const std::string& mission_id, int priority);
QueuedMissionStartDecision DecideQueuedMissionStart(
    const QueuedMissionStartRequest& request);
bool MissionBelongsToWorkflowOrder(
    const std::string& mission_id, const std::string& order_id);
bool MissionBelongsToStationOrderBatch(
    const std::string& mission_id, const std::string& batch_id);
std::optional<std::string> PausedWorkflowForMission(
    const std::string& mission_id, const std::unordered_set<std::string>& paused_order_ids);
std::vector<std::string> QueuedWorkflowMissionIds(
    const std::vector<QueuedMission>& queue, const std::string& order_id);
std::size_t RemoveQueuedWorkflowMissions(
    std::vector<QueuedMission>& queue, const std::string& order_id,
    std::vector<std::string>* removed_mission_ids);
std::size_t RemoveQueuedStationOrderBatchMissions(
    std::vector<QueuedMission>& queue, const std::string& batch_id,
    std::vector<std::string>* removed_mission_ids);
std::vector<std::string> ReprioritizeQueuedMissionBatch(
    std::vector<QueuedMission>& queue, const std::vector<std::string>& mission_ids,
    const std::vector<int>& priorities);
std::optional<std::size_t> FindNextRunnableMissionIndex(
    const std::vector<QueuedMission>& queue,
    const std::unordered_set<std::string>& paused_order_ids);
MissionQueueDispatchResult PopNextRunnableMission(
    std::vector<QueuedMission> queue,
    const std::unordered_set<std::string>& paused_order_ids);
MissionQueueAdmissionDecision PlanMissionQueueAdmission(
    const MissionQueueAdmissionRequest& request);
MissionSubmitGateDecision PlanMissionSubmitGate(
    const MissionSubmitGateRequest& request);
QueuedMissionCancelDecision PlanQueuedMissionCancel(
    const QueuedMissionCancelRequest& request);
StationOrderBatchCancelDecision PlanStationOrderBatchCancel(
    const StationOrderBatchCancelRequest& request);
StationOrderBatchSubmitPlan PlanStationOrderBatchSubmit(
    const StationOrderBatchSubmitRequest& request);
StationOrderBatchSubmitSummary PlanStationOrderBatchSubmitSummary(
    const std::string& batch_id, bool continue_on_error, int accepted_count,
    int rejected_count);
QueuedMissionReprioritizeDecision PlanQueuedMissionReprioritize(
    const QueuedMissionReprioritizeRequest& request);
QueuedMissionReprioritizeDecision PlanQueuedMissionBatchReprioritize(
    const QueuedMissionBatchReprioritizeRequest& request);

}  // namespace robot_tasks
