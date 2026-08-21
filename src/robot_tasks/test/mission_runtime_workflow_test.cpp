#include "robot_tasks/mission_runtime_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(MissionRuntimeWorkflowTest, AutostartIdle_StartsDefaultMission) {
  MissionRuntimeTickInput input;
  input.autostart = true;
  input.autostart_sent = false;
  input.mission_active = false;

  const auto decision = DecideMissionRuntimeTick(input);

  EXPECT_TRUE(decision.start_default_mission);
  EXPECT_FALSE(decision.start_queued_mission);
  EXPECT_FALSE(decision.clear_queue_start_request);
  EXPECT_EQ(decision.branch, "autostart");
}

TEST(MissionRuntimeWorkflowTest, QueueStartIdle_StartsQueuedMissionAndClearsRequest) {
  MissionRuntimeTickInput input;
  input.queue_start_requested = true;
  input.mission_active = false;

  const auto decision = DecideMissionRuntimeTick(input);

  EXPECT_FALSE(decision.start_default_mission);
  EXPECT_TRUE(decision.start_queued_mission);
  EXPECT_TRUE(decision.clear_queue_start_request);
  EXPECT_EQ(decision.branch, "queue_start");
}

TEST(MissionRuntimeWorkflowTest, AutostartAndQueueRequested_StagesBothRequests) {
  MissionRuntimeTickInput input;
  input.autostart = true;
  input.queue_start_requested = true;
  input.mission_active = false;

  const auto decision = DecideMissionRuntimeTick(input);

  EXPECT_TRUE(decision.start_default_mission);
  EXPECT_TRUE(decision.start_queued_mission);
  EXPECT_TRUE(decision.clear_queue_start_request);
  EXPECT_EQ(decision.branch, "autostart_then_queue_if_still_idle");
}

TEST(MissionRuntimeWorkflowTest, MissionActive_DoesNotStartNewMission) {
  MissionRuntimeTickInput input;
  input.autostart = true;
  input.queue_start_requested = true;
  input.mission_active = true;

  const auto decision = DecideMissionRuntimeTick(input);

  EXPECT_FALSE(decision.start_default_mission);
  EXPECT_FALSE(decision.start_queued_mission);
  EXPECT_FALSE(decision.clear_queue_start_request);
  EXPECT_EQ(decision.branch, "publish_only");
}

TEST(MissionRuntimeWorkflowTest, AutostartAlreadySent_DoesNotRestartDefaultMission) {
  MissionRuntimeTickInput input;
  input.autostart = true;
  input.autostart_sent = true;
  input.mission_active = false;

  const auto decision = DecideMissionRuntimeTick(input);

  EXPECT_FALSE(decision.start_default_mission);
  EXPECT_FALSE(decision.start_queued_mission);
  EXPECT_FALSE(decision.clear_queue_start_request);
  EXPECT_EQ(decision.branch, "publish_only");
}

}  // namespace robot_tasks
