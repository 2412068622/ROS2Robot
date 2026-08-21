#include "robot_tasks/mission_result_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(MissionResultBehaviorTreeTest, CancellationRequested_SelectsCanceledBranch) {
  MissionResultWorkflowInput input;
  input.code = MissionActionResultCode::kSucceeded;
  input.result_success = true;
  input.cancellation_requested = true;

  const auto result = TickMissionResultBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "canceled");
  EXPECT_TRUE(result.decision.clear_cancellation_requested);
  EXPECT_TRUE(result.decision.release_all_resources);
}

TEST(MissionResultBehaviorTreeTest, CanceledActionResult_SelectsCanceledBranch) {
  MissionResultWorkflowInput input;
  input.code = MissionActionResultCode::kCanceled;

  const auto result = TickMissionResultBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "canceled");
  EXPECT_EQ(result.decision.state, "CANCELED");
  EXPECT_FALSE(result.decision.handle_failure);
}

TEST(MissionResultBehaviorTreeTest, SuccessfulActionResult_SelectsSucceededBranch) {
  MissionResultWorkflowInput input;
  input.code = MissionActionResultCode::kSucceeded;
  input.result_success = true;

  const auto result = TickMissionResultBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "succeeded");
  EXPECT_TRUE(result.decision.release_all_resources);
  EXPECT_FALSE(result.decision.release_route_locks);
  EXPECT_TRUE(result.decision.mark_resource_occupied);
}

TEST(MissionResultBehaviorTreeTest, SuccessfulActionWithFailedPayload_SelectsFailedBranch) {
  MissionResultWorkflowInput input;
  input.code = MissionActionResultCode::kSucceeded;
  input.result_success = false;

  const auto result = TickMissionResultBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "failed");
  EXPECT_TRUE(result.decision.handle_failure);
  EXPECT_TRUE(result.decision.release_all_resources);
}

TEST(MissionResultBehaviorTreeTest, FailedActionResult_SelectsFailedBranch) {
  MissionResultWorkflowInput input;
  input.code = MissionActionResultCode::kFailed;
  input.result_success = false;

  const auto result = TickMissionResultBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "failed");
  EXPECT_TRUE(result.decision.handle_failure);
  EXPECT_TRUE(result.decision.release_all_resources);
}

}  // namespace robot_tasks
