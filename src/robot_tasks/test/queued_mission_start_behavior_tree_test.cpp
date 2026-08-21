#include "robot_tasks/queued_mission_start_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

QueuedMissionStartBehaviorTreeInput MakeInput(
    const bool start_if_idle, const bool preempt_current, const bool mission_active) {
  QueuedMissionStartBehaviorTreeInput input;
  input.request.start_if_idle = start_if_idle;
  input.request.preempt_current = preempt_current;
  input.request.mission_active = mission_active;
  return input;
}

}  // namespace

TEST(QueuedMissionStartBehaviorTreeTest, ActivePreemptRequested_CancelsActiveAndStartsQueue) {
  const auto result = TickQueuedMissionStartBehaviorTree(MakeInput(false, true, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "preempt_active");
  EXPECT_TRUE(result.decision.run_queue);
  EXPECT_TRUE(result.decision.request_queue_start);
  EXPECT_TRUE(result.decision.cancel_active);
}

TEST(QueuedMissionStartBehaviorTreeTest, IdleStartRequested_StartsQueueWithoutCancel) {
  const auto result = TickQueuedMissionStartBehaviorTree(MakeInput(true, false, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "start_idle");
  EXPECT_TRUE(result.decision.run_queue);
  EXPECT_TRUE(result.decision.request_queue_start);
  EXPECT_FALSE(result.decision.cancel_active);
}

TEST(QueuedMissionStartBehaviorTreeTest, ActiveWithoutPreempt_KeepsMissionQueued) {
  const auto result = TickQueuedMissionStartBehaviorTree(MakeInput(true, false, true));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "keep_queued");
  EXPECT_FALSE(result.decision.run_queue);
  EXPECT_FALSE(result.decision.request_queue_start);
  EXPECT_FALSE(result.decision.cancel_active);
}

TEST(QueuedMissionStartBehaviorTreeTest, IdlePreemptOnly_DoesNotStartWithoutStartFlag) {
  const auto result = TickQueuedMissionStartBehaviorTree(MakeInput(false, true, false));

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "keep_queued");
  EXPECT_FALSE(result.decision.run_queue);
  EXPECT_FALSE(result.decision.request_queue_start);
  EXPECT_FALSE(result.decision.cancel_active);
}

}  // namespace robot_tasks
