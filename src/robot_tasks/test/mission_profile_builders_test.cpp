#include "robot_tasks/mission_profile_builders.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

StationCatalog ExampleStationCatalog() {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"receiving", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"storage_a", "map", 2.0, 0.0, 0.25});
  catalog.stations.push_back(Station{"packing", "map", 3.0, 1.0, 1.57});
  catalog.edges.push_back(StationEdge{"receiving", "storage_a", 2.0, true, true});
  catalog.edges.push_back(StationEdge{"storage_a", "packing", 1.5, true, true});
  return catalog;
}

}  // namespace

TEST(MissionProfileBuildersTest, BuildTwoWaypointProfile_EmptyFrame_DefaultsToMap) {
  const auto profile = BuildTwoWaypointProfile("transport_1", "", 1.0, 2.0, 0.1, 3.0, 4.0, 0.2);

  EXPECT_EQ(profile.mission_id, "transport_1");
  EXPECT_EQ(profile.frame_id, "map");
  EXPECT_FALSE(profile.loop);
  ASSERT_EQ(profile.waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(profile.waypoints[0].x, 1.0);
  EXPECT_DOUBLE_EQ(profile.waypoints[1].yaw, 0.2);
}

TEST(MissionProfileBuildersTest, BuildStationWaypointProfile_EmptyFrame_DefaultsToMap) {
  const Station station{"charger", "", -1.0, 2.5, -0.5};

  const auto profile = BuildStationWaypointProfile("charge_1", station);

  EXPECT_EQ(profile.frame_id, "map");
  ASSERT_EQ(profile.waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(profile.waypoints[0].x, -1.0);
  EXPECT_DOUBLE_EQ(profile.waypoints[0].yaw, -0.5);
}

TEST(MissionProfileBuildersTest, ResolveStationPair_UnknownPickup_ReturnsMessage) {
  std::string message;

  const auto pair = ResolveStationPair(ExampleStationCatalog(), "missing", "packing", &message);

  EXPECT_FALSE(pair.has_value());
  EXPECT_EQ(message, "unknown pickup station: missing");
}

TEST(MissionProfileBuildersTest, ResolveStationPair_MixedFrames_ReturnsMessage) {
  auto catalog = ExampleStationCatalog();
  catalog.stations.push_back(Station{"lab", "lab_map", 4.0, 0.0, 0.0});
  std::string message;

  const auto pair = ResolveStationPair(catalog, "receiving", "lab", &message);

  EXPECT_FALSE(pair.has_value());
  EXPECT_EQ(message, "pickup and dropoff stations use different frames: map vs lab_map");
}

TEST(MissionProfileBuildersTest, BuildStationSequenceProfiles_ValidRoute_BuildsLegs) {
  std::string message;

  const auto legs = BuildStationSequenceProfiles(
      ExampleStationCatalog(), {"receiving", "storage_a", "packing"}, "station_sequence_order_1",
      &message);

  ASSERT_TRUE(legs.has_value()) << message;
  ASSERT_EQ(legs->size(), 2U);
  EXPECT_EQ((*legs)[0].profile.mission_id, "station_sequence_order_1_leg_1");
  EXPECT_EQ((*legs)[0].pickup_id, "receiving");
  EXPECT_EQ((*legs)[0].dropoff_id, "storage_a");
  EXPECT_EQ((*legs)[1].profile.mission_id, "station_sequence_order_1_leg_2");
}

TEST(MissionProfileBuildersTest, BuildStationSequenceProfiles_MissingRoute_ReturnsMessage) {
  auto catalog = ExampleStationCatalog();
  catalog.edges.clear();
  std::string message;

  const auto legs =
      BuildStationSequenceProfiles(catalog, {"receiving", "storage_a"}, "station_sequence_bad", &message);

  EXPECT_FALSE(legs.has_value());
  EXPECT_EQ(message, "no enabled station route from receiving to storage_a");
}

TEST(MissionProfileBuildersTest, BuildStationSequenceEstimateProfile_MixedFrames_ReturnsMessage) {
  auto catalog = ExampleStationCatalog();
  catalog.stations.push_back(Station{"lab", "lab_map", 4.0, 0.0, 0.0});
  std::string message;

  const auto profile =
      BuildStationSequenceEstimateProfile(catalog, {"receiving", "lab"}, &message);

  EXPECT_FALSE(profile.has_value());
  EXPECT_EQ(message, "station sequence contains mixed frames");
}

}  // namespace robot_tasks
