#include "robot_tasks/mission_queue_behavior_tree.hpp"

#include <gtest/gtest.h>

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

TEST(MissionQueueBehaviorTreeTest, RunnableQueue_PopsFirstRunnableMission) {
  MissionQueueBehaviorTreeInput input;
  input.queue = {
      MakeQueuedMission("station_order_fleet_paused_order_step_0", 10, 0),
      MakeQueuedMission("delivery_b", 2, 1),
  };
  input.paused_order_ids = {"paused_order"};

  const auto result = TickMissionQueueBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "mission_queue_dispatch");
  ASSERT_TRUE(result.dispatch.mission.has_value());
  EXPECT_EQ(result.dispatch.mission->profile.mission_id, "delivery_b");
  ASSERT_EQ(result.dispatch.remaining_queue.size(), 1U);
  EXPECT_EQ(
      result.dispatch.remaining_queue[0].profile.mission_id,
      "station_order_fleet_paused_order_step_0");
}

TEST(MissionQueueBehaviorTreeTest, EmptyQueue_ReturnsQueueEmptyFailure) {
  const auto result = TickMissionQueueBehaviorTree(MissionQueueBehaviorTreeInput{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.dispatch.queue_empty);
  EXPECT_EQ(result.dispatch.message, "mission queue is empty");
  EXPECT_TRUE(result.dispatch.remaining_queue.empty());
}

TEST(MissionQueueBehaviorTreeTest, AllPausedQueue_ReturnsAllPausedFailure) {
  MissionQueueBehaviorTreeInput input;
  input.queue = {MakeQueuedMission("station_sequence_paused_order_step_0", 10, 0)};
  input.paused_order_ids = {"paused_order"};

  const auto result = TickMissionQueueBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.dispatch.all_paused);
  EXPECT_EQ(result.dispatch.message, "all queued workflow missions are paused");
  ASSERT_EQ(result.dispatch.remaining_queue.size(), 1U);
  EXPECT_EQ(result.dispatch.remaining_queue[0].profile.mission_id, input.queue[0].profile.mission_id);
}

}  // namespace robot_tasks
