#include "robot_tasks/mission_queue.hpp"

#include <gtest/gtest.h>

#include <unordered_set>

namespace robot_tasks {
namespace {

QueuedMission MakeQueuedMission(
    const std::string& mission_id, const int priority, const std::uint64_t sequence) {
  QueuedMission mission;
  mission.profile.mission_id = mission_id;
  mission.mission_file = mission_id + ".yaml";
  mission.priority = priority;
  mission.sequence = sequence;
  return mission;
}

}  // namespace

TEST(MissionQueueTest, SortMissionQueue_HigherPriorityFirstAndStableWithinPriority) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("low", 1, 0),
      MakeQueuedMission("high_late", 5, 2),
      MakeQueuedMission("high_early", 5, 1),
  };

  SortMissionQueue(queue);

  ASSERT_EQ(queue.size(), 3U);
  EXPECT_EQ(queue[0].profile.mission_id, "high_early");
  EXPECT_EQ(queue[1].profile.mission_id, "high_late");
  EXPECT_EQ(queue[2].profile.mission_id, "low");
}

TEST(MissionQueueTest, MissionIdQueued_ExistingMission_ReturnsTrue) {
  const std::vector<QueuedMission> queue{
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };

  EXPECT_TRUE(MissionIdQueued(queue, "delivery_b"));
  EXPECT_FALSE(MissionIdQueued(queue, "missing"));
}

TEST(MissionQueueTest, RemoveQueuedMissionById_DuplicateIds_RemovesAllMatches) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("delivery_a", 3, 2),
  };

  const auto removed = RemoveQueuedMissionById(queue, "delivery_a");

  EXPECT_EQ(removed, 2U);
  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue[0].profile.mission_id, "delivery_b");
}

TEST(MissionQueueTest, ReprioritizeQueuedMission_ExistingMission_UpdatesPriorityOnly) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };

  EXPECT_TRUE(ReprioritizeQueuedMission(queue, "delivery_a", 9));
  EXPECT_EQ(queue[0].priority, 9);
  EXPECT_EQ(queue[1].priority, 2);
  EXPECT_FALSE(ReprioritizeQueuedMission(queue, "missing", 5));
}

TEST(MissionQueueTest, DecideQueuedMissionStart_IdleStartRequested_StartsQueue) {
  const auto decision = DecideQueuedMissionStart(
      QueuedMissionStartRequest{true, false, false});

  EXPECT_TRUE(decision.run_queue);
  EXPECT_TRUE(decision.request_queue_start);
  EXPECT_FALSE(decision.cancel_active);
}

TEST(MissionQueueTest, DecideQueuedMissionStart_ActivePreemptRequested_StartsAfterCancel) {
  const auto decision = DecideQueuedMissionStart(
      QueuedMissionStartRequest{false, true, true});

  EXPECT_TRUE(decision.run_queue);
  EXPECT_TRUE(decision.request_queue_start);
  EXPECT_TRUE(decision.cancel_active);
}

TEST(MissionQueueTest, DecideQueuedMissionStart_ActiveWithoutPreempt_DoesNotStartQueue) {
  const auto decision = DecideQueuedMissionStart(
      QueuedMissionStartRequest{true, false, true});

  EXPECT_FALSE(decision.run_queue);
  EXPECT_FALSE(decision.request_queue_start);
  EXPECT_FALSE(decision.cancel_active);
}

TEST(MissionQueueTest, DecideQueuedMissionStart_IdlePreemptOnly_DoesNotOverrideStartFlag) {
  const auto decision = DecideQueuedMissionStart(
      QueuedMissionStartRequest{false, true, false});

  EXPECT_FALSE(decision.run_queue);
  EXPECT_FALSE(decision.request_queue_start);
  EXPECT_FALSE(decision.cancel_active);
}

TEST(MissionQueueTest, PlanMissionQueueAdmission_IdleRunner_PlansQueuedState) {
  MissionQueueAdmissionRequest request;
  request.mission_active = false;
  request.queue_size = 2;
  request.message = "queued station transport order station_order_a priority 10";

  const auto decision = PlanMissionQueueAdmission(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.queue_size, 2);
  EXPECT_EQ(decision.message, request.message);
  EXPECT_TRUE(decision.set_state);
  EXPECT_EQ(decision.state, "QUEUED");
  EXPECT_EQ(decision.state_message, request.message);
  EXPECT_TRUE(decision.state_recoverable);
}

TEST(MissionQueueTest, PlanMissionQueueAdmission_ActiveRunner_DoesNotPlanStateChange) {
  MissionQueueAdmissionRequest request;
  request.mission_active = true;
  request.queue_size = 1;
  request.message = "queued charging request charging_request_a using charger charger_a";

  const auto decision = PlanMissionQueueAdmission(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.queue_size, 1);
  EXPECT_EQ(decision.message, request.message);
  EXPECT_FALSE(decision.set_state);
}

TEST(MissionQueueTest, PlanMissionQueueAdmission_StateDisabled_DoesNotPlanStateChange) {
  MissionQueueAdmissionRequest request;
  request.mission_active = false;
  request.queue_size = 1;
  request.message = "preempt mission queued to start";
  request.set_state_when_idle = false;

  const auto decision = PlanMissionQueueAdmission(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.queue_size, 1);
  EXPECT_EQ(decision.message, request.message);
  EXPECT_FALSE(decision.set_state);
}

TEST(MissionQueueTest, PlanMissionSubmitGate_DuplicateRejectsBeforePreflight) {
  MissionSubmitGateRequest request;
  request.mission_id = "delivery_a";
  request.duplicate_message_prefix = "transport order";
  request.preflight_message_prefix = "transport order";
  request.mission_already_active_or_queued = true;
  request.preflight.allowed = false;
  request.preflight.message = "would fail if evaluated first";

  const auto decision = PlanMissionSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "delivery_a");
  EXPECT_EQ(decision.reject_reason, "duplicate");
  EXPECT_EQ(decision.message, "transport order already active or queued: delivery_a");
}

TEST(MissionQueueTest, PlanMissionSubmitGate_PreflightRejectsWithPrefix) {
  MissionSubmitGateRequest request;
  request.mission_id = "delivery_a";
  request.duplicate_message_prefix = "mission";
  request.preflight_message_prefix = "mission";
  request.preflight.allowed = false;
  request.preflight.message = "battery insufficient";

  const auto decision = PlanMissionSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "delivery_a");
  EXPECT_EQ(decision.reject_reason, "preflight");
  EXPECT_EQ(decision.message, "mission preflight rejected: battery insufficient");
}

TEST(MissionQueueTest, PlanMissionSubmitGate_AcceptsValidMission) {
  MissionSubmitGateRequest request;
  request.mission_id = "delivery_a";

  const auto decision = PlanMissionSubmitGate(request);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "delivery_a");
  EXPECT_EQ(decision.reject_reason, "none");
  EXPECT_TRUE(decision.message.empty());
}

TEST(MissionQueueTest, PlanQueuedMissionCancel_QueuedMission_RemovesAndPlansResourceRelease) {
  QueuedMissionCancelRequest request;
  request.queue = {
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };
  request.mission_id = "delivery_a";
  request.mission_active = false;

  const auto decision = PlanQueuedMissionCancel(request);

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.active_cancel_requested);
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
  ASSERT_EQ(decision.release_mission_ids.size(), 1U);
  EXPECT_EQ(decision.release_mission_ids[0], "delivery_a");
  EXPECT_TRUE(decision.set_state);
  EXPECT_EQ(decision.state_message, "removed queued mission delivery_a");
  EXPECT_EQ(decision.queue_size, 1);
}

TEST(MissionQueueTest, PlanQueuedMissionCancel_ActiveMission_RequestsActiveCancelOnly) {
  QueuedMissionCancelRequest request;
  request.queue = {MakeQueuedMission("delivery_b", 2, 1)};
  request.mission_id = "delivery_a";
  request.cancel_active = true;
  request.mission_active = true;
  request.active_mission_id = "delivery_a";

  const auto decision = PlanQueuedMissionCancel(request);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.active_cancel_requested);
  EXPECT_TRUE(decision.release_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
  EXPECT_FALSE(decision.set_state);
  EXPECT_EQ(decision.message, "active mission cancel requested: delivery_a");
}

TEST(MissionQueueTest, PlanQueuedMissionCancel_MissingMission_ReturnsNotFoundMessage) {
  QueuedMissionCancelRequest request;
  request.queue = {MakeQueuedMission("delivery_b", 2, 1)};
  request.mission_id = "missing";
  request.cancel_active = true;

  const auto decision = PlanQueuedMissionCancel(request);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.active_cancel_requested);
  EXPECT_EQ(decision.message, "mission not queued or active: missing");
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
}

TEST(MissionQueueTest, PlanStationOrderBatchSubmit_BuildsStationOrderItemPlans) {
  StationOrderBatchSubmitRequest request;
  request.batch_id = "batch_7";
  request.original_order_ids = {"alpha", "beta"};
  request.sanitized_order_ids = {"alpha", "beta"};
  request.pickup_station_ids = {"receiving", "storage"};
  request.dropoff_station_ids = {"packing", "dock"};
  request.priority = 30;
  request.start_if_idle = true;
  request.preempt_current = true;

  const auto plan = PlanStationOrderBatchSubmit(request);

  EXPECT_TRUE(plan.accepted);
  ASSERT_EQ(plan.items.size(), 2U);
  EXPECT_TRUE(plan.items[0].accepted);
  EXPECT_EQ(plan.items[0].station_order_id, "batch_batch_7_order_alpha");
  EXPECT_EQ(plan.items[0].pickup_station, "receiving");
  EXPECT_EQ(plan.items[0].dropoff_station, "packing");
  EXPECT_EQ(plan.items[0].priority, 30);
  EXPECT_TRUE(plan.items[0].start_if_idle);
  EXPECT_TRUE(plan.items[0].preempt_current);
  EXPECT_EQ(plan.items[1].station_order_id, "batch_batch_7_order_beta");
  EXPECT_EQ(plan.items[1].priority, 29);
  EXPECT_FALSE(plan.items[1].start_if_idle);
  EXPECT_FALSE(plan.items[1].preempt_current);
}

TEST(MissionQueueTest, PlanStationOrderBatchSubmit_EmptyBatchIdRejects) {
  StationOrderBatchSubmitRequest request;
  request.original_order_ids = {"alpha"};
  request.sanitized_order_ids = {"alpha"};
  request.pickup_station_ids = {"receiving"};
  request.dropoff_station_ids = {"packing"};

  const auto plan = PlanStationOrderBatchSubmit(request);

  EXPECT_FALSE(plan.accepted);
  EXPECT_EQ(plan.message, "batch_id is empty");
  EXPECT_TRUE(plan.items.empty());
}

TEST(MissionQueueTest, PlanStationOrderBatchSubmit_SizeMismatchRejects) {
  StationOrderBatchSubmitRequest request;
  request.batch_id = "batch_7";
  request.original_order_ids = {"alpha"};
  request.sanitized_order_ids = {"alpha"};
  request.pickup_station_ids = {"receiving", "extra"};
  request.dropoff_station_ids = {"packing"};

  const auto plan = PlanStationOrderBatchSubmit(request);

  EXPECT_FALSE(plan.accepted);
  EXPECT_EQ(
      plan.message,
      "batch order_ids, pickup_station_ids and dropoff_station_ids size mismatch");
}

TEST(MissionQueueTest, PlanStationOrderBatchSubmit_EmptyOrderStopsWhenContinueDisabled) {
  StationOrderBatchSubmitRequest request;
  request.batch_id = "batch_7";
  request.original_order_ids = {"", "beta"};
  request.sanitized_order_ids = {"", "beta"};
  request.pickup_station_ids = {"receiving", "storage"};
  request.dropoff_station_ids = {"packing", "dock"};
  request.continue_on_error = false;

  const auto plan = PlanStationOrderBatchSubmit(request);

  EXPECT_TRUE(plan.accepted);
  ASSERT_EQ(plan.items.size(), 1U);
  EXPECT_FALSE(plan.items[0].accepted);
  EXPECT_EQ(plan.items[0].reject_order_id, "");
  EXPECT_EQ(plan.items[0].reject_message, "order_id is empty");
}

TEST(MissionQueueTest, PlanStationOrderBatchSubmitSummary_ContinueOnErrorAllowsPartialSuccess) {
  const auto strict = PlanStationOrderBatchSubmitSummary("batch_7", false, 1, 1);
  const auto partial = PlanStationOrderBatchSubmitSummary("batch_7", true, 1, 1);

  EXPECT_FALSE(strict.success);
  EXPECT_TRUE(partial.success);
  EXPECT_EQ(partial.accepted_count, 1);
  EXPECT_EQ(partial.rejected_count, 1);
  EXPECT_EQ(partial.message, "station order batch batch_7 accepted=1 rejected=1");
}

TEST(MissionQueueTest, PlanStationOrderBatchCancel_QueuedBatch_RemovesAndPlansRelease) {
  StationOrderBatchCancelRequest request;
  request.queue = {
      MakeQueuedMission("station_order_batch_batch_7_order_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("station_order_batch_batch_7_order_b", 3, 2),
  };
  request.batch_id = "batch_7";

  const auto decision = PlanStationOrderBatchCancel(request);

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.active_cancel_requested);
  ASSERT_EQ(decision.canceled_mission_ids.size(), 2U);
  EXPECT_EQ(decision.canceled_mission_ids[0], "station_order_batch_batch_7_order_a");
  EXPECT_EQ(decision.canceled_mission_ids[1], "station_order_batch_batch_7_order_b");
  EXPECT_EQ(decision.release_mission_ids, decision.canceled_mission_ids);
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
  EXPECT_EQ(decision.queue_size, 1);
  EXPECT_TRUE(decision.set_state);
  EXPECT_EQ(decision.message, "canceled station order batch batch_7 mission_count=2");
}

TEST(MissionQueueTest, PlanStationOrderBatchCancel_ActiveBatchMission_RequestsActiveCancelOnly) {
  StationOrderBatchCancelRequest request;
  request.queue = {MakeQueuedMission("delivery_b", 2, 1)};
  request.batch_id = "batch_7";
  request.cancel_active = true;
  request.mission_active = true;
  request.active_mission_id = "station_order_batch_batch_7_order_a";

  const auto decision = PlanStationOrderBatchCancel(request);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.active_cancel_requested);
  ASSERT_EQ(decision.canceled_mission_ids.size(), 1U);
  EXPECT_EQ(decision.canceled_mission_ids[0], "station_order_batch_batch_7_order_a");
  EXPECT_TRUE(decision.release_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
  EXPECT_EQ(decision.queue_size, 1);
  EXPECT_FALSE(decision.set_state);
  EXPECT_EQ(decision.message, "canceled station order batch batch_7 mission_count=1");
}

TEST(MissionQueueTest, PlanStationOrderBatchCancel_MissingBatch_ReturnsNotFound) {
  StationOrderBatchCancelRequest request;
  request.queue = {MakeQueuedMission("station_order_batch_other_order_a", 1, 0)};
  request.batch_id = "batch_7";
  request.cancel_active = true;
  request.mission_active = true;
  request.active_mission_id = "station_order_batch_other_order_b";

  const auto decision = PlanStationOrderBatchCancel(request);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.active_cancel_requested);
  EXPECT_TRUE(decision.canceled_mission_ids.empty());
  EXPECT_TRUE(decision.release_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "station_order_batch_other_order_a");
  EXPECT_EQ(
      decision.message,
      "station order batch has no queued or active mission: batch_7");
}

TEST(MissionQueueTest, PlanQueuedMissionReprioritize_MatchingMission_SortsAndPlansState) {
  QueuedMissionReprioritizeRequest request;
  request.queue = {
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };
  request.mission_id = "delivery_a";
  request.priority = 9;

  const auto decision = PlanQueuedMissionReprioritize(request);

  EXPECT_TRUE(decision.success);
  ASSERT_EQ(decision.updated_mission_ids.size(), 1U);
  EXPECT_EQ(decision.updated_mission_ids[0], "delivery_a");
  ASSERT_EQ(decision.remaining_queue.size(), 2U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_a");
  EXPECT_EQ(decision.remaining_queue[0].priority, 9);
  EXPECT_TRUE(decision.set_state);
  EXPECT_EQ(decision.message, "reprioritized queued mission delivery_a to 9");
}

TEST(MissionQueueTest, PlanQueuedMissionReprioritize_MissingMission_DoesNotMutateQueue) {
  QueuedMissionReprioritizeRequest request;
  request.queue = {MakeQueuedMission("delivery_b", 2, 1)};
  request.mission_id = "missing";
  request.priority = 9;
  request.mission_active = true;

  const auto decision = PlanQueuedMissionReprioritize(request);

  EXPECT_FALSE(decision.success);
  EXPECT_TRUE(decision.updated_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_b");
  EXPECT_EQ(decision.message, "mission not queued: missing");
  EXPECT_FALSE(decision.set_state);
}

TEST(MissionQueueTest, PlanQueuedMissionBatchReprioritize_Mismatch_RejectsWithoutSorting) {
  QueuedMissionBatchReprioritizeRequest request;
  request.queue = {
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };
  request.mission_ids = {"delivery_a"};
  request.priorities = {9, 8};

  const auto decision = PlanQueuedMissionBatchReprioritize(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "mission_ids and priorities size mismatch");
  EXPECT_TRUE(decision.updated_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 2U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_a");
  EXPECT_EQ(decision.remaining_queue[1].profile.mission_id, "delivery_b");
}

TEST(MissionQueueTest, PlanQueuedMissionBatchReprioritize_PartialUpdate_ReturnsFailureAndSortedQueue) {
  QueuedMissionBatchReprioritizeRequest request;
  request.queue = {
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("delivery_c", 3, 2),
  };
  request.mission_ids = {"delivery_a", "missing", "delivery_b"};
  request.priorities = {9, 8, 7};

  const auto decision = PlanQueuedMissionBatchReprioritize(request);

  EXPECT_FALSE(decision.success);
  ASSERT_EQ(decision.updated_mission_ids.size(), 2U);
  EXPECT_EQ(decision.updated_mission_ids[0], "delivery_a");
  EXPECT_EQ(decision.updated_mission_ids[1], "delivery_b");
  ASSERT_EQ(decision.remaining_queue.size(), 3U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_a");
  EXPECT_EQ(decision.remaining_queue[1].profile.mission_id, "delivery_b");
  EXPECT_EQ(decision.remaining_queue[2].profile.mission_id, "delivery_c");
  EXPECT_TRUE(decision.set_state);
  EXPECT_EQ(decision.message, "reprioritized 2 queued mission(s)");
}

TEST(MissionQueueTest, MissionBelongsToWorkflowOrder_KnownPrefixes_MatchesOrderId) {
  EXPECT_TRUE(MissionBelongsToWorkflowOrder("station_order_fleet_order_42_step_0", "order_42"));
  EXPECT_TRUE(MissionBelongsToWorkflowOrder("facility_action_order_42_step_1", "order_42"));
  EXPECT_TRUE(MissionBelongsToWorkflowOrder("station_sequence_order_42_step_2", "order_42"));
  EXPECT_FALSE(MissionBelongsToWorkflowOrder("station_order_batch_order_42_order_0", "order_42"));
  EXPECT_FALSE(MissionBelongsToWorkflowOrder("station_order_fleet_other_step_0", "order_42"));
}

TEST(MissionQueueTest, MissionBelongsToStationOrderBatch_BatchPrefix_MatchesBatchId) {
  EXPECT_TRUE(MissionBelongsToStationOrderBatch("station_order_batch_batch_7_order_0", "batch_7"));
  EXPECT_TRUE(MissionBelongsToStationOrderBatch("station_order_batch_batch_7_order_12", "batch_7"));
  EXPECT_FALSE(MissionBelongsToStationOrderBatch("station_order_batch_other_order_0", "batch_7"));
  EXPECT_FALSE(MissionBelongsToStationOrderBatch("station_sequence_batch_7_step_0", "batch_7"));
}

TEST(MissionQueueTest, QueuedWorkflowMissionIds_MatchingOrder_ReturnsQueuedIdsInOrder) {
  const std::vector<QueuedMission> queue{
      MakeQueuedMission("station_order_fleet_order_42_step_0", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("facility_action_order_42_step_1", 3, 2),
  };

  const auto ids = QueuedWorkflowMissionIds(queue, "order_42");

  ASSERT_EQ(ids.size(), 2U);
  EXPECT_EQ(ids[0], "station_order_fleet_order_42_step_0");
  EXPECT_EQ(ids[1], "facility_action_order_42_step_1");
}

TEST(MissionQueueTest, RemoveQueuedWorkflowMissions_MatchingOrder_RemovesAndReturnsIds) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("station_order_fleet_order_42_step_0", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("station_sequence_order_42_step_1", 3, 2),
  };
  std::vector<std::string> removed_ids;

  const auto removed = RemoveQueuedWorkflowMissions(queue, "order_42", &removed_ids);

  EXPECT_EQ(removed, 2U);
  ASSERT_EQ(removed_ids.size(), 2U);
  EXPECT_EQ(removed_ids[0], "station_order_fleet_order_42_step_0");
  EXPECT_EQ(removed_ids[1], "station_sequence_order_42_step_1");
  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue[0].profile.mission_id, "delivery_b");
}

TEST(MissionQueueTest, RemoveQueuedStationOrderBatchMissions_MatchingBatch_RemovesAndReturnsIds) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("station_order_batch_batch_7_order_0", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("station_order_batch_batch_7_order_1", 3, 2),
  };
  std::vector<std::string> removed_ids;

  const auto removed = RemoveQueuedStationOrderBatchMissions(queue, "batch_7", &removed_ids);

  EXPECT_EQ(removed, 2U);
  ASSERT_EQ(removed_ids.size(), 2U);
  EXPECT_EQ(removed_ids[0], "station_order_batch_batch_7_order_0");
  EXPECT_EQ(removed_ids[1], "station_order_batch_batch_7_order_1");
  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue[0].profile.mission_id, "delivery_b");
}

TEST(MissionQueueTest, ReprioritizeQueuedMissionBatch_MatchingIds_UpdatesExistingOnly) {
  std::vector<QueuedMission> queue{
      MakeQueuedMission("delivery_a", 1, 0),
      MakeQueuedMission("delivery_b", 2, 1),
      MakeQueuedMission("delivery_c", 3, 2),
  };

  const auto updated = ReprioritizeQueuedMissionBatch(
      queue, {"delivery_c", "missing", "delivery_a"}, {9, 8, 7});

  ASSERT_EQ(updated.size(), 2U);
  EXPECT_EQ(updated[0], "delivery_c");
  EXPECT_EQ(updated[1], "delivery_a");
  EXPECT_EQ(queue[0].priority, 7);
  EXPECT_EQ(queue[1].priority, 2);
  EXPECT_EQ(queue[2].priority, 9);
}

TEST(MissionQueueTest, FindNextRunnableMissionIndex_SkipsPausedWorkflowMissions) {
  const std::vector<QueuedMission> queue{
      MakeQueuedMission("station_order_fleet_paused_order_step_0", 10, 0),
      MakeQueuedMission("station_sequence_paused_order_step_1", 9, 1),
      MakeQueuedMission("delivery_b", 2, 2),
  };
  const std::unordered_set<std::string> paused_orders{"paused_order"};

  const auto next = FindNextRunnableMissionIndex(queue, paused_orders);

  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(*next, 2U);
}

TEST(MissionQueueTest, FindNextRunnableMissionIndex_AllPaused_ReturnsNullopt) {
  const std::vector<QueuedMission> queue{
      MakeQueuedMission("station_order_fleet_paused_order_step_0", 10, 0),
      MakeQueuedMission("station_sequence_paused_order_step_1", 9, 1),
  };
  const std::unordered_set<std::string> paused_orders{"paused_order"};

  EXPECT_FALSE(FindNextRunnableMissionIndex(queue, paused_orders).has_value());
  EXPECT_EQ(PausedWorkflowForMission(queue[0].profile.mission_id, paused_orders), "paused_order");
}

}  // namespace robot_tasks
