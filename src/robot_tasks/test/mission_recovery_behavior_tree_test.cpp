#include "robot_tasks/mission_recovery_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

MissionProfile MakeProfile(const std::string& mission_id) {
  MissionProfile profile;
  profile.mission_id = mission_id;
  return profile;
}

MissionRecoveryBehaviorTreeInput MakeInput(const std::string& policy) {
  MissionRecoveryBehaviorTreeInput input;
  input.config.policy = policy;
  input.config.retry_limit = 1;
  input.retry_count = 0;
  input.mission_id = "delivery_a";
  input.failure_message = "planner failed";
  input.active_profile = MakeProfile("delivery_a");
  input.mission_file = "delivery_a.yaml";
  input.recovery_priority = 90;
  input.next_queue_sequence = 12;
  return input;
}

}  // namespace

TEST(MissionRecoveryBehaviorTreeTest, RetryPolicy_StagesRetryMissionAndQueueStart) {
  const auto result = TickMissionRecoveryBehaviorTree(MakeInput("retry_then_dock"));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "retry");
  EXPECT_EQ(result.state, "RECOVERING");
  EXPECT_TRUE(result.recoverable);
  EXPECT_EQ(result.retry_count, 1);
  EXPECT_EQ(result.next_queue_sequence, 13U);
  EXPECT_TRUE(result.queue_running);
  EXPECT_TRUE(result.queue_start_requested);
  ASSERT_TRUE(result.retry_mission.has_value());
  EXPECT_EQ(result.retry_mission->profile.mission_id, "delivery_a");
  EXPECT_EQ(result.retry_mission->mission_file, "delivery_a.yaml");
  EXPECT_EQ(result.retry_mission->priority, 90);
  EXPECT_EQ(result.retry_mission->sequence, 12U);
  EXPECT_NE(result.state_message.find("planner failed"), std::string::npos);
}

TEST(MissionRecoveryBehaviorTreeTest, RetryWithoutActiveProfile_FallsBackToFailedBranch) {
  auto input = MakeInput("retry_then_dock");
  input.active_profile.reset();

  const auto result = TickMissionRecoveryBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "failed");
  EXPECT_EQ(result.state, "FAILED");
  EXPECT_TRUE(result.stop_queue);
  EXPECT_FALSE(result.retry_mission.has_value());
  EXPECT_EQ(result.retry_count, 0);
  EXPECT_EQ(result.next_queue_sequence, 12U);
}

TEST(MissionRecoveryBehaviorTreeTest, RetryLimitReached_RequestsDockRecovery) {
  auto input = MakeInput("retry_then_dock");
  input.retry_count = 1;

  const auto result = TickMissionRecoveryBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "dock");
  EXPECT_EQ(result.state, "RECOVERING");
  EXPECT_TRUE(result.request_dock_return);
  EXPECT_FALSE(result.retry_mission.has_value());
}

TEST(MissionRecoveryBehaviorTreeTest, DockMissionFailure_RequestsManualIntervention) {
  auto input = MakeInput("retry_then_dock");
  input.is_dock_mission = true;

  const auto result = TickMissionRecoveryBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "manual");
  EXPECT_EQ(result.state, "NEEDS_OPERATOR");
  EXPECT_TRUE(result.stop_queue);
  EXPECT_FALSE(result.recoverable);
}

TEST(MissionRecoveryBehaviorTreeTest, DisabledRecovery_UsesFailedBranch) {
  auto input = MakeInput("disabled");

  const auto result = TickMissionRecoveryBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "failed");
  EXPECT_EQ(result.state, "FAILED");
  EXPECT_EQ(result.decision.action, "none");
  EXPECT_TRUE(result.stop_queue);
  EXPECT_FALSE(result.recoverable);
}

}  // namespace robot_tasks
