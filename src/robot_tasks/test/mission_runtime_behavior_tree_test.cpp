#include "robot_tasks/mission_runtime_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

MissionRuntimeBehaviorTreeInput MakeInput(
    const bool autostart, const bool autostart_sent, const bool mission_active,
    const bool queue_start_requested) {
  MissionRuntimeBehaviorTreeInput input;
  input.tick.autostart = autostart;
  input.tick.autostart_sent = autostart_sent;
  input.tick.mission_active = mission_active;
  input.tick.queue_start_requested = queue_start_requested;
  return input;
}

}  // namespace

TEST(MissionRuntimeBehaviorTreeTest, AutostartIdle_SelectsAutostartBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, false, false, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "autostart");
  EXPECT_TRUE(result.decision.start_default_mission);
  EXPECT_FALSE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, QueueStartIdle_SelectsQueueStartBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(false, false, false, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "queue_start");
  EXPECT_FALSE(result.decision.start_default_mission);
  EXPECT_TRUE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, AutostartAndQueueRequested_SelectsCombinedBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, false, false, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "autostart_then_queue_if_still_idle");
  EXPECT_TRUE(result.decision.start_default_mission);
  EXPECT_TRUE(result.decision.start_queued_mission);
}

TEST(MissionRuntimeBehaviorTreeTest, NoRuntimeStartNeeded_SelectsPublishOnlyBranch) {
  const auto result = TickMissionRuntimeBehaviorTree(MakeInput(true, true, false, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "publish_only");
  EXPECT_FALSE(result.decision.start_default_mission);
  EXPECT_FALSE(result.decision.start_queued_mission);
}

}  // namespace robot_tasks
