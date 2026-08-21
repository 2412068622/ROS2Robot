#include "robot_tasks/workflow_control.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

QueuedMission MakeQueuedMission(const std::string& mission_id) {
  QueuedMission mission;
  mission.profile.mission_id = mission_id;
  mission.mission_file = mission_id + ".yaml";
  return mission;
}

MissionEvent MakeEvent(const std::string& mission_id, const std::string& state) {
  MissionEvent event;
  event.mission_id = mission_id;
  event.state = state;
  return event;
}

}  // namespace

TEST(WorkflowControlTest, PlanWorkflowCancel_RemovesQueuedAndRequestsActiveCancel) {
  WorkflowCancelRequest request;
  request.order_id = "order_42";
  request.cancel_active = true;
  request.queue = {
      MakeQueuedMission("station_order_fleet_order_42_step_0"),
      MakeQueuedMission("delivery_other"),
      MakeQueuedMission("facility_action_order_42_step_1"),
  };
  request.active.active = true;
  request.active.mission_id = "station_sequence_order_42_step_2";

  const auto decision = PlanWorkflowCancel(request);

  ASSERT_TRUE(decision.success);
  EXPECT_TRUE(decision.active_cancel_requested);
  EXPECT_EQ(
      decision.canceled_mission_ids,
      (std::vector<std::string>{
          "station_order_fleet_order_42_step_0",
          "facility_action_order_42_step_1",
          "station_sequence_order_42_step_2"}));
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.remaining_queue[0].profile.mission_id, "delivery_other");
  EXPECT_EQ(decision.message, "canceled workflow order_42 mission_count=3");
}

TEST(WorkflowControlTest, PlanWorkflowCancel_NoMatches_ReturnsStableFailure) {
  WorkflowCancelRequest request;
  request.order_id = "missing";
  request.cancel_active = true;
  request.queue = {MakeQueuedMission("delivery_other")};
  request.active.active = true;
  request.active.mission_id = "station_sequence_order_42_step_0";

  const auto decision = PlanWorkflowCancel(request);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.active_cancel_requested);
  EXPECT_TRUE(decision.canceled_mission_ids.empty());
  ASSERT_EQ(decision.remaining_queue.size(), 1U);
  EXPECT_EQ(decision.message, "workflow has no queued or active mission: missing");
}

TEST(WorkflowControlTest, PlanWorkflowPause_QueuesOrderAndActivePause) {
  WorkflowPauseRequest request;
  request.order_id = "order_42";
  request.pause_active = true;
  request.queue = {MakeQueuedMission("station_order_fleet_order_42_step_0")};
  request.active.active = true;
  request.active.mission_id = "facility_action_order_42_step_1";

  const auto decision = PlanWorkflowPause(request);

  ASSERT_TRUE(decision.success);
  EXPECT_TRUE(decision.active_pause_requested);
  EXPECT_TRUE(decision.runner_paused);
  EXPECT_TRUE(decision.clear_queue_start_requested);
  EXPECT_EQ(
      decision.affected_mission_ids,
      (std::vector<std::string>{
          "station_order_fleet_order_42_step_0",
          "facility_action_order_42_step_1"}));
  EXPECT_EQ(decision.paused_order_ids.count("order_42"), 1U);
}

TEST(WorkflowControlTest, PlanWorkflowResume_RemovesPausedOrderAndStartsIdleQueue) {
  WorkflowResumeRequest request;
  request.order_id = "order_42";
  request.queue = {MakeQueuedMission("station_sequence_order_42_step_0")};
  request.paused_order_ids = {"order_42"};

  const auto decision = PlanWorkflowResume(request);

  ASSERT_TRUE(decision.success);
  EXPECT_FALSE(decision.active_resume_requested);
  EXPECT_TRUE(decision.queue_running);
  EXPECT_TRUE(decision.queue_start_requested);
  EXPECT_EQ(decision.paused_order_ids.count("order_42"), 0U);
  EXPECT_EQ(decision.affected_mission_ids,
            (std::vector<std::string>{"station_sequence_order_42_step_0"}));
}

TEST(WorkflowControlTest, PlanWorkflowResume_ActivePausedWorkflowRequestsResume) {
  WorkflowResumeRequest request;
  request.order_id = "order_42";
  request.resume_active = true;
  request.active.active = true;
  request.active.paused = true;
  request.active.mission_id = "facility_action_order_42_step_0";

  const auto decision = PlanWorkflowResume(request);

  ASSERT_TRUE(decision.success);
  EXPECT_TRUE(decision.active_resume_requested);
  EXPECT_FALSE(decision.runner_paused);
  EXPECT_FALSE(decision.queue_running);
  EXPECT_EQ(decision.affected_mission_ids,
            (std::vector<std::string>{"facility_action_order_42_step_0"}));
}

TEST(WorkflowControlTest, BuildWorkflowStatusSnapshot_ReportsPausedBeforeRunning) {
  WorkflowStatusRequest request;
  request.order_id = "order_42";
  request.current_state = "RUNNING";
  request.queue = {MakeQueuedMission("station_order_fleet_order_42_step_1")};
  request.active.active = true;
  request.active.mission_id = "station_sequence_order_42_step_0";
  request.paused_order_ids = {"order_42"};
  request.events = {MakeEvent("facility_action_order_42_step_2", "FINISHED")};

  const auto snapshot = BuildWorkflowStatusSnapshot(request);

  ASSERT_TRUE(snapshot.success);
  EXPECT_TRUE(snapshot.paused);
  EXPECT_EQ(snapshot.state, "PAUSED");
  EXPECT_EQ(snapshot.running_mission_id, "station_sequence_order_42_step_0");
  ASSERT_EQ(snapshot.queued_mission_ids.size(), 1U);
  ASSERT_EQ(snapshot.finished_mission_ids.size(), 1U);
  EXPECT_EQ(snapshot.total_steps, 3);
  EXPECT_EQ(snapshot.finished_steps, 1);
  EXPECT_EQ(snapshot.queued_steps, 1);
}

TEST(WorkflowControlTest, BuildWorkflowStatusSnapshot_AllFinishedReportsFinished) {
  WorkflowStatusRequest request;
  request.order_id = "order_42";
  request.events = {
      MakeEvent("station_order_fleet_order_42_step_0", "FINISHED"),
      MakeEvent("station_sequence_order_42_step_1", "FINISHED"),
  };

  const auto snapshot = BuildWorkflowStatusSnapshot(request);

  ASSERT_TRUE(snapshot.success);
  EXPECT_EQ(snapshot.state, "FINISHED");
  EXPECT_EQ(snapshot.total_steps, 2);
  EXPECT_EQ(snapshot.finished_steps, 2);
}

}  // namespace robot_tasks
