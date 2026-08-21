#include "robot_tasks/mission_event_log.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

MissionEvent Event(const std::string& mission_id, const std::string& state) {
  MissionEvent event;
  event.stamp = mission_id + "_" + state;
  event.mission_id = mission_id;
  event.state = state;
  event.previous_state = "PREVIOUS";
  event.message = "message";
  event.recoverable = state != "FAILED";
  return event;
}

}  // namespace

TEST(MissionEventLogTest, NormalizeEventLimit_DefaultsAndCaps) {
  EXPECT_EQ(NormalizeEventLimit(0, 20U, 200U), 20U);
  EXPECT_EQ(NormalizeEventLimit(-5, 20U, 200U), 20U);
  EXPECT_EQ(NormalizeEventLimit(7, 20U, 200U), 7U);
  EXPECT_EQ(NormalizeEventLimit(500, 20U, 200U), 200U);
}

TEST(MissionEventLogTest, TrimMissionEvents_RemovesOldestEvents) {
  std::vector<MissionEvent> events{
      Event("m1", "QUEUED"), Event("m2", "RUNNING"), Event("m3", "FINISHED")};

  const auto removed = TrimMissionEvents(events, 2U);

  EXPECT_EQ(removed, 1U);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].mission_id, "m2");
  EXPECT_EQ(events[1].mission_id, "m3");
}

TEST(MissionEventLogTest, TrimMissionEvents_WithinLimit_RemovesNothing) {
  std::vector<MissionEvent> events{Event("m1", "QUEUED")};

  EXPECT_EQ(TrimMissionEvents(events, 2U), 0U);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].mission_id, "m1");
}

TEST(MissionEventLogTest, AppendMissionEvent_AddsEventAndTrimsOldest) {
  std::vector<MissionEvent> events{Event("m1", "QUEUED"), Event("m2", "RUNNING")};

  AppendMissionEvent(
      events,
      MakeMissionEvent("stamp", "m3", "FINISHED", "RUNNING", "mission finished", true), 2U);

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].mission_id, "m2");
  EXPECT_EQ(events[1].mission_id, "m3");
  EXPECT_EQ(events[1].message, "mission finished");
}

TEST(MissionEventLogTest, SelectRecentMissionEvents_ReturnsNewestFirst) {
  const std::vector<MissionEvent> events{
      Event("m1", "QUEUED"), Event("m2", "RUNNING"), Event("m3", "FINISHED")};

  const auto selected = SelectRecentMissionEvents(events, 2U);

  ASSERT_EQ(selected.size(), 2U);
  EXPECT_EQ(selected[0].mission_id, "m3");
  EXPECT_EQ(selected[1].mission_id, "m2");
}

TEST(MissionEventLogTest, SelectRecentMissionEvents_FiltersByStateAndMissionId) {
  const std::vector<MissionEvent> events{
      Event("m1", "FAILED"), Event("m2", "FAILED"), Event("m2", "FINISHED")};

  const auto selected = SelectRecentMissionEvents(events, 5U, "FAILED", "m2");

  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected[0].mission_id, "m2");
  EXPECT_EQ(selected[0].state, "FAILED");
}

}  // namespace robot_tasks
