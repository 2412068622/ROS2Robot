#include "robot_tasks/station_catalog.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(StationCatalogTest, BuildStationListProjection_PreservesStationFields) {
  StationCatalog catalog;
  catalog.stations.push_back(Station{"receiving", "map", 1.0, 2.0, 0.25});
  catalog.stations.push_back(Station{"storage", "odom", -3.0, 4.5, -1.5});

  const auto projection = BuildStationListProjection(catalog);

  EXPECT_EQ(projection.station_ids, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.frame_ids, (std::vector<std::string>{"map", "odom"}));
  EXPECT_EQ(projection.x, (std::vector<double>{1.0, -3.0}));
  EXPECT_EQ(projection.y, (std::vector<double>{2.0, 4.5}));
  EXPECT_EQ(projection.yaw, (std::vector<double>{0.25, -1.5}));
  EXPECT_EQ(projection.message, "loaded 2 station(s)");
}

TEST(StationCatalogTest, BuildStationRouteListProjection_PreservesRouteFields) {
  StationCatalog catalog;
  catalog.edges.push_back(StationEdge{"receiving", "storage", 5.5, true, true});
  catalog.edges.push_back(StationEdge{"storage", "dock", 2.0, false, false});

  const auto projection = BuildStationRouteListProjection(catalog);

  EXPECT_EQ(projection.from_station, (std::vector<std::string>{"receiving", "storage"}));
  EXPECT_EQ(projection.to_station, (std::vector<std::string>{"storage", "dock"}));
  EXPECT_EQ(projection.distance_m, (std::vector<double>{5.5, 2.0}));
  EXPECT_EQ(projection.bidirectional, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{true, false}));
  EXPECT_EQ(projection.message, "loaded 2 route edge(s)");
}

}  // namespace
}  // namespace robot_tasks
