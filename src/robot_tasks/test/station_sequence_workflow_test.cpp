#include "robot_tasks/station_sequence_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

StationSequenceLeg MakeLeg(const std::string& mission_id) {
  StationSequenceLeg leg;
  leg.profile.mission_id = mission_id;
  leg.profile.frame_id = "map";
  leg.profile.waypoints = {MissionWaypoint{0.0, 0.0, 0.0}, MissionWaypoint{1.0, 0.0, 0.0}};
  leg.pickup_id = "pickup";
  leg.dropoff_id = "dropoff";
  return leg;
}

}  // namespace

TEST(StationSequenceWorkflowTest, CheckStationSequenceConflicts_KnownMission_RejectsLeg) {
  const std::vector<StationSequenceLeg> legs{
      MakeLeg("station_sequence_order_1_leg_1"),
      MakeLeg("station_sequence_order_1_leg_2"),
  };

  const auto result = CheckStationSequenceConflicts(
      legs, {"station_sequence_order_1_leg_2", "other_mission"});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
      result.message,
      "station sequence leg already active or queued: station_sequence_order_1_leg_2");
}

TEST(StationSequenceWorkflowTest, CheckStationSequenceConflicts_NoConflict_AcceptsSequence) {
  const std::vector<StationSequenceLeg> legs{MakeLeg("station_sequence_order_1_leg_1")};

  const auto result = CheckStationSequenceConflicts(legs, {"other_mission"});

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.message, "station sequence accepted");
}

TEST(StationSequenceWorkflowTest, CheckStationSequenceLegPreflight_RejectedLeg_AddsContext) {
  auto preflight = MissionPreflightResult{};
  preflight.allowed = false;
  preflight.message = "battery below minimum";

  const auto result =
      CheckStationSequenceLegPreflight(MakeLeg("station_sequence_order_1_leg_1"), preflight);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
      result.message,
      "station sequence preflight rejected for station_sequence_order_1_leg_1: "
      "battery below minimum");
}

TEST(StationSequenceWorkflowTest, StationSequenceMissionIds_MultipleLegs_ReturnsIdsInOrder) {
  const auto mission_ids = StationSequenceMissionIds(
      {MakeLeg("station_sequence_order_1_leg_1"), MakeLeg("station_sequence_order_1_leg_2")});

  ASSERT_EQ(mission_ids.size(), 2U);
  EXPECT_EQ(mission_ids[0], "station_sequence_order_1_leg_1");
  EXPECT_EQ(mission_ids[1], "station_sequence_order_1_leg_2");
}

TEST(StationSequenceWorkflowTest, BuildStationSequenceQueuedMissions_PreservesPriorityAndSequence) {
  const auto queued = BuildStationSequenceQueuedMissions(
      {MakeLeg("station_sequence_order_1_leg_1"), MakeLeg("station_sequence_order_1_leg_2")}, 7,
      42);

  ASSERT_EQ(queued.size(), 2U);
  EXPECT_EQ(queued[0].profile.mission_id, "station_sequence_order_1_leg_1");
  EXPECT_EQ(queued[0].mission_file, "station_sequence:station_sequence_order_1_leg_1");
  EXPECT_EQ(queued[0].priority, 7);
  EXPECT_EQ(queued[0].sequence, 42U);
  EXPECT_EQ(queued[1].sequence, 43U);
}

TEST(StationSequenceWorkflowTest, BuildStationSequenceQueuedMessage_AnyCount_FormatsSummary) {
  EXPECT_EQ(BuildStationSequenceQueuedMessage(3), "queued station sequence with 3 leg(s)");
}

}  // namespace robot_tasks
