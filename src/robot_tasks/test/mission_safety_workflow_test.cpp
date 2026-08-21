#include "robot_tasks/mission_safety_workflow.hpp"

#include <gtest/gtest.h>

namespace {

robot_tasks::MissionSafetyState ClearState() {
  robot_tasks::MissionSafetyState state;
  state.zone_speed_limit_mps = 0.6;
  state.message = "safety ok";
  return state;
}

robot_tasks::MissionSafetyState BlockedState() {
  auto state = ClearState();
  state.obstacle_blocked = true;
  state.blockage_id = "pallet_aisle";
  state.blockage_paused_mission = true;
  state.stop_reason = "pallet blocking aisle";
  state.message = "pallet blocking aisle";
  state.mission_id = "mission_alpha";
  return state;
}

}  // namespace

TEST(MissionSafetyWorkflowTest, DynamicSpeedLimitRejectsNegativeWithoutMutation) {
  const auto decision = robot_tasks::PlanSetDynamicSpeedLimit(
      robot_tasks::MissionSafetySpeedLimitRequest{ClearState(), -0.1, "bad"});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.response_message, "speed_limit_mps must be >= 0");
  EXPECT_DOUBLE_EQ(decision.response_effective_speed_limit_mps, 0.6);
}

TEST(MissionSafetyWorkflowTest, DynamicSpeedLimitSetsAndClearsRuntimeLimit) {
  auto decision = robot_tasks::PlanSetDynamicSpeedLimit(
      robot_tasks::MissionSafetySpeedLimitRequest{ClearState(), 0.2, "narrow aisle"});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.response_state, "SPEED_LIMITED");
  EXPECT_DOUBLE_EQ(decision.response_runtime_speed_limit_mps, 0.2);
  EXPECT_DOUBLE_EQ(decision.response_effective_speed_limit_mps, 0.2);
  EXPECT_EQ(decision.event.state, "DYNAMIC_SPEED_LIMIT");

  decision = robot_tasks::PlanSetDynamicSpeedLimit(
      robot_tasks::MissionSafetySpeedLimitRequest{decision.safety, 0.0, "clear aisle"});

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.response_state, "OK");
  EXPECT_EQ(decision.response_message, "dynamic speed limit cleared");
  EXPECT_DOUBLE_EQ(decision.response_effective_speed_limit_mps, 0.6);
}

TEST(MissionSafetyWorkflowTest, ReportObstaclePausesOnlyWhenRunnerIsNotAlreadyPaused) {
  const auto decision = robot_tasks::PlanReportObstacleBlockage(
      robot_tasks::MissionSafetyReportBlockageRequest{
          ClearState(), "pallet_aisle", "pallet blocking aisle", "mission_alpha", "RUNNING",
          true, true, true, false});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_TRUE(decision.safety.obstacle_blocked);
  EXPECT_TRUE(decision.safety.blockage_paused_mission);
  EXPECT_TRUE(decision.side_effects.request_pause);
  EXPECT_EQ(decision.side_effects.runner_state, "SAFETY_BLOCKED");
  EXPECT_EQ(decision.event.mission_id, "safety_pallet_aisle");
  EXPECT_EQ(decision.response_state, "OBSTACLE_BLOCKED");
}

TEST(MissionSafetyWorkflowTest, ReportObstacleDoesNotClaimOperatorPausedMission) {
  const auto decision = robot_tasks::PlanReportObstacleBlockage(
      robot_tasks::MissionSafetyReportBlockageRequest{
          ClearState(), "pallet_aisle", "pallet blocking aisle", "mission_alpha", "PAUSED",
          true, true, true, true});

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.safety.blockage_paused_mission);
  EXPECT_FALSE(decision.side_effects.request_pause);
}

TEST(MissionSafetyWorkflowTest, ReportObstacleRejectsDuplicateActiveBlockage) {
  const auto decision = robot_tasks::PlanReportObstacleBlockage(
      robot_tasks::MissionSafetyReportBlockageRequest{
          BlockedState(), "fallen_box", "box in aisle", "mission_beta", "RUNNING",
          false, true, true, false});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.safety.blockage_paused_mission);
  EXPECT_EQ(decision.safety.blockage_id, "pallet_aisle");
  EXPECT_EQ(decision.response_message, "obstacle blockage already active: pallet_aisle");
}

TEST(MissionSafetyWorkflowTest, ClearObstacleMismatchRejectsWithoutMutation) {
  const auto decision = robot_tasks::PlanClearObstacleBlockage(
      robot_tasks::MissionSafetyClearBlockageRequest{
          BlockedState(), "wrong", "clear", "SAFETY_BLOCKED", true, true, true});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.safety.obstacle_blocked);
  EXPECT_EQ(decision.response_message, "active blockage mismatch: pallet_aisle");
}

TEST(MissionSafetyWorkflowTest, ClearObstacleResumesOnlyIfSafetyPausedMission) {
  const auto decision = robot_tasks::PlanClearObstacleBlockage(
      robot_tasks::MissionSafetyClearBlockageRequest{
          BlockedState(), "pallet_aisle", "operator removed pallet", "SAFETY_BLOCKED",
          true, true, true});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_FALSE(decision.safety.obstacle_blocked);
  EXPECT_FALSE(decision.safety.blockage_paused_mission);
  EXPECT_TRUE(decision.side_effects.request_resume);
  EXPECT_EQ(decision.side_effects.runner_state, "RUNNING");
  EXPECT_EQ(decision.event.state, "OBSTACLE_CLEARED");
  EXPECT_EQ(decision.response_state, "OK");
}

TEST(MissionSafetyWorkflowTest, ClearObstacleDoesNotResumeOperatorPausedMission) {
  auto state = BlockedState();
  state.blockage_paused_mission = false;

  const auto decision = robot_tasks::PlanClearObstacleBlockage(
      robot_tasks::MissionSafetyClearBlockageRequest{
          state, "pallet_aisle", "operator removed pallet", "SAFETY_BLOCKED",
          true, true, true});

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.side_effects.request_resume);
  EXPECT_FALSE(decision.side_effects.set_runner_state);
}

TEST(MissionSafetyWorkflowTest, ClearObstacleIdleSafetyBlockedReturnsReady) {
  const auto decision = robot_tasks::PlanClearObstacleBlockage(
      robot_tasks::MissionSafetyClearBlockageRequest{
          BlockedState(), "pallet_aisle", "operator removed pallet", "SAFETY_BLOCKED",
          true, false, false});

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.side_effects.request_resume);
  EXPECT_TRUE(decision.side_effects.set_runner_state);
  EXPECT_EQ(decision.side_effects.runner_state, "READY");
}
