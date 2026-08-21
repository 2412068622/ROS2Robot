#include "robot_tasks/mission_localization_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(MissionLocalizationWorkflowTest, HighCovarianceMarksLostAndRequestsPause) {
  MissionLocalizationPoseUpdateRequest request;
  request.current.state = "OK";
  request.covariance_x = 1.2;
  request.covariance_y = 1.1;
  request.covariance_threshold = 0.5;
  request.pause_on_lost = true;
  request.mission_active = true;
  request.runner_paused = false;

  const auto decision = PlanLocalizationPoseUpdate(request);

  EXPECT_FALSE(decision.localization.localized);
  EXPECT_EQ(decision.localization.state, "LOST");
  EXPECT_TRUE(decision.localization.paused_mission);
  ASSERT_TRUE(decision.event.emit);
  EXPECT_EQ(decision.event.state, "LOCALIZATION_LOST");
  EXPECT_EQ(decision.event.previous_state, "OK");
  EXPECT_TRUE(decision.side_effects.request_pause);
  EXPECT_EQ(decision.side_effects.runner_state, "LOCALIZATION_LOST");
}

TEST(MissionLocalizationWorkflowTest, HighCovarianceDoesNotOwnAnAlreadyPausedMission) {
  MissionLocalizationPoseUpdateRequest request;
  request.current.state = "OK";
  request.covariance_x = 1.2;
  request.covariance_y = 1.1;
  request.covariance_threshold = 0.5;
  request.pause_on_lost = true;
  request.mission_active = true;
  request.runner_paused = true;

  const auto decision = PlanLocalizationPoseUpdate(request);

  EXPECT_FALSE(decision.localization.paused_mission);
  EXPECT_FALSE(decision.side_effects.request_pause);
}

TEST(MissionLocalizationWorkflowTest, RecoveredCovarianceResumesOnlyLocalizationPausedMission) {
  MissionLocalizationPoseUpdateRequest request;
  request.current.state = "LOST";
  request.current.localized = false;
  request.current.paused_mission = true;
  request.covariance_x = 0.02;
  request.covariance_y = 0.03;
  request.covariance_threshold = 0.5;
  request.mission_active = true;
  request.runner_paused = true;

  const auto decision = PlanLocalizationPoseUpdate(request);

  EXPECT_TRUE(decision.localization.localized);
  EXPECT_EQ(decision.localization.state, "RECOVERED");
  EXPECT_FALSE(decision.localization.paused_mission);
  ASSERT_TRUE(decision.event.emit);
  EXPECT_EQ(decision.event.state, "LOCALIZATION_RECOVERED");
  EXPECT_EQ(decision.event.previous_state, "LOST");
  EXPECT_TRUE(decision.side_effects.request_resume);
  EXPECT_EQ(decision.side_effects.runner_state, "RUNNING");
}

TEST(MissionLocalizationWorkflowTest, RecoveredCovarianceDoesNotResumeOperatorPausedMission) {
  MissionLocalizationPoseUpdateRequest request;
  request.current.state = "LOST";
  request.current.localized = false;
  request.current.paused_mission = false;
  request.covariance_x = 0.02;
  request.covariance_y = 0.03;
  request.covariance_threshold = 0.5;
  request.mission_active = true;
  request.runner_paused = true;

  const auto decision = PlanLocalizationPoseUpdate(request);

  EXPECT_EQ(decision.localization.state, "RECOVERED");
  EXPECT_FALSE(decision.side_effects.request_resume);
}

TEST(MissionLocalizationWorkflowTest, RelocalizationRequestRecordsCurrentState) {
  MissionRelocalizationRequest request;
  request.current.state = "LOST";
  request.current.paused_mission = true;
  request.reason = "operator requested reset";

  const auto decision = PlanRelocalizationRequest(request);

  EXPECT_FALSE(decision.localization.localized);
  EXPECT_EQ(decision.localization.state, "RELOCALIZING");
  EXPECT_TRUE(decision.localization.paused_mission);
  ASSERT_TRUE(decision.event.emit);
  EXPECT_EQ(decision.event.state, "RELOCALIZING");
  EXPECT_EQ(decision.event.previous_state, "LOST");
  EXPECT_EQ(decision.event.message, "operator requested reset");
}

TEST(MissionLocalizationWorkflowTest, InitialPoseRecoveryResumesLocalizationPausedMission) {
  MissionInitialPoseRecoveryRequest request;
  request.current.state = "RELOCALIZING";
  request.current.paused_mission = true;
  request.station_id = "dock";
  request.mission_active = true;
  request.runner_paused = true;

  const auto decision = PlanInitialPoseRecovery(request);

  EXPECT_TRUE(decision.localization.localized);
  EXPECT_EQ(decision.localization.state, "RECOVERED");
  EXPECT_EQ(decision.localization.station_id, "dock");
  EXPECT_FALSE(decision.localization.paused_mission);
  ASSERT_TRUE(decision.event.emit);
  EXPECT_EQ(decision.event.state, "LOCALIZATION_RECOVERED");
  EXPECT_EQ(decision.event.previous_state, "RELOCALIZING");
  EXPECT_TRUE(decision.side_effects.request_resume);
}

}  // namespace robot_tasks
