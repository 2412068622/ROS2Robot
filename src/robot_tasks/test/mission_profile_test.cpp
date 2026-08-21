#include "robot_tasks/mission_profile.hpp"
#include "robot_tasks/dock_catalog.hpp"
#include "robot_tasks/facility_catalog.hpp"
#include "robot_tasks/fleet_catalog.hpp"
#include "robot_tasks/mission_estimator.hpp"
#include "robot_tasks/mission_preflight.hpp"
#include "robot_tasks/mission_recovery_policy.hpp"
#include "robot_tasks/scenario_catalog.hpp"
#include "robot_tasks/station_catalog.hpp"
#include "robot_navigation/zone_catalog.hpp"

#include <filesystem>
#include <fstream>
#include <unistd.h>

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

class MissionProfileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("robot_mission_profile_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  std::filesystem::path WriteMission(const std::string& content) const {
    const auto path = test_dir_ / "mission.yaml";
    std::ofstream out(path);
    out << content;
    return path;
  }

  std::filesystem::path test_dir_;
};

}  // namespace

TEST_F(MissionProfileTest, LoadMissionProfile_ValidMission_ReturnsWaypoints) {
  const auto path = WriteMission(
      "mission_id: patrol\nframe_id: map\nloop: true\ngoals:\n"
      "  - x: 1.0\n    y: 2.0\n    yaw: 0.5\n"
      "  - x: -1.0\n    y: 0.0\n    yaw: -0.2\n");

  const auto profile = LoadMissionProfile(path);

  ASSERT_TRUE(profile.has_value());
  EXPECT_EQ(profile->mission_id, "patrol");
  EXPECT_EQ(profile->frame_id, "map");
  EXPECT_TRUE(profile->loop);
  ASSERT_EQ(profile->waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(profile->waypoints[0].x, 1.0);
  EXPECT_DOUBLE_EQ(profile->waypoints[0].y, 2.0);
  EXPECT_DOUBLE_EQ(profile->waypoints[0].yaw, 0.5);
}

TEST_F(MissionProfileTest, LoadMissionProfile_MissingMissionId_UsesFileStem) {
  const auto path = WriteMission("frame_id: map\ngoals:\n  - x: 1.0\n    y: 2.0\n");

  const auto profile = LoadMissionProfile(path);

  ASSERT_TRUE(profile.has_value());
  EXPECT_EQ(profile->mission_id, "mission");
}

TEST_F(MissionProfileTest, LoadMissionProfile_NoWaypoints_ReturnsNullopt) {
  const auto path = WriteMission("mission_id: empty\nframe_id: map\ngoals:\n");

  EXPECT_FALSE(LoadMissionProfile(path).has_value());
}

TEST(MissionProfileValidationTest, ValidateMissionProfile_EmptyFrameId_ReturnsFalse) {
  MissionProfile profile;
  profile.mission_id = "bad";
  profile.frame_id = "";
  profile.waypoints.push_back(MissionWaypoint{});
  std::string message;

  EXPECT_FALSE(ValidateMissionProfile(profile, &message));
  EXPECT_EQ(message, "frame_id is empty");
}

TEST(MissionEstimateTest, EstimateMissionCost_TwoSegments_ReturnsDistanceEtaAndBattery) {
  MissionProfile profile;
  profile.mission_id = "estimate";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{3.0, 4.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{6.0, 8.0, 0.0});

  const auto estimate = EstimateMissionCost(profile, 2.0, 24.0, 0.1, 22.0);

  EXPECT_EQ(estimate.waypoint_count, 3);
  EXPECT_EQ(estimate.distance_source, "waypoint_polyline");
  EXPECT_DOUBLE_EQ(estimate.distance_m, 10.0);
  EXPECT_DOUBLE_EQ(estimate.eta_sec, 5.0);
  EXPECT_DOUBLE_EQ(estimate.battery_drop_v, 1.0);
  EXPECT_DOUBLE_EQ(estimate.projected_battery_voltage, 23.0);
  EXPECT_TRUE(estimate.battery_sufficient);
}

TEST(MissionEstimateTest, EstimateMissionCost_LowProjectedBattery_ReturnsInsufficient) {
  MissionProfile profile;
  profile.mission_id = "estimate";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{10.0, 0.0, 0.0});

  const auto estimate = EstimateMissionCost(profile, 1.0, 22.5, 0.1, 22.0);

  EXPECT_DOUBLE_EQ(estimate.distance_m, 10.0);
  EXPECT_DOUBLE_EQ(estimate.projected_battery_voltage, 21.5);
  EXPECT_FALSE(estimate.battery_sufficient);
}

TEST(MissionEstimateTest, EstimateMissionPathDistance_TwoSegments_ReturnsPolylineLength) {
  MissionProfile profile;
  profile.mission_id = "estimate";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{0.0, 3.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{4.0, 3.0, 0.0});

  EXPECT_DOUBLE_EQ(EstimateMissionPathDistance(profile), 7.0);
}

TEST(MissionEstimateTest, EstimateDistanceCost_PlannedPathSource_ReturnsSource) {
  const auto estimate =
      EstimateDistanceCost(12.0, 2, 3.0, 24.0, 0.1, 22.0, "nav2_path");

  EXPECT_EQ(estimate.distance_source, "nav2_path");
  EXPECT_DOUBLE_EQ(estimate.distance_m, 12.0);
  EXPECT_DOUBLE_EQ(estimate.eta_sec, 4.0);
}

TEST_F(MissionProfileTest, ValidateStationTransportPreflight_PathOverride_UsesPlannedDistance) {
  MissionProfile profile;
  profile.mission_id = "estimate_station";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{3.0, 0.0, 0.0});
  StationCatalog catalog;
  catalog.stations.push_back(Station{"pickup", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"dropoff", "map", 3.0, 0.0, 0.0});
  catalog.edges.push_back(StationEdge{"pickup", "dropoff", 30.0, true, true});
  MissionPreflightConfig config;
  config.enabled = false;
  config.nominal_speed_mps = 2.0;
  const robot_navigation::ZoneCatalog zone_catalog(test_dir_ / "unused_zones.yaml");

  const auto preflight = ValidateStationTransportPreflight(
      profile, catalog, "pickup", "dropoff", zone_catalog, config, 12.0);

  EXPECT_TRUE(preflight.allowed);
  EXPECT_DOUBLE_EQ(preflight.cost.distance_m, 12.0);
  EXPECT_DOUBLE_EQ(preflight.cost.eta_sec, 6.0);
  EXPECT_EQ(preflight.cost.distance_source, "planned_path");
}

TEST_F(MissionProfileTest, ValidateMissionPreflight_WaypointInKeepout_ReturnsRejected) {
  const auto zones_path = test_dir_ / "zones.yaml";
  std::ofstream zones(zones_path);
  zones << "zones:\n"
        << "  - zone_id: blocked\n"
        << "    map_name: indoor_room\n"
        << "    type: keepout\n"
        << "    frame_id: map\n"
        << "    enabled: true\n"
        << "    polygon:\n"
        << "      - [0.0, 0.0]\n"
        << "      - [2.0, 0.0]\n"
        << "      - [2.0, 2.0]\n"
        << "      - [0.0, 2.0]\n";
  zones.close();

  MissionProfile profile;
  profile.mission_id = "blocked_mission";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{1.0, 1.0, 0.0});
  MissionPreflightConfig config;

  const auto result =
      ValidateMissionPreflight(profile, robot_navigation::ZoneCatalog(zones_path), config);

  EXPECT_FALSE(result.allowed);
  EXPECT_NE(result.message.find("keepout"), std::string::npos);
}

TEST_F(MissionProfileTest, ValidateMissionPreflight_LowBattery_ReturnsRejected) {
  const auto zones_path = test_dir_ / "zones.yaml";
  std::ofstream zones(zones_path);
  zones << "zones:\n";
  zones.close();

  MissionProfile profile;
  profile.mission_id = "low_battery_mission";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{10.0, 0.0, 0.0});
  MissionPreflightConfig config;
  config.battery_voltage = 22.2;
  config.battery_drop_per_meter = 0.1;
  config.minimum_battery_voltage = 22.0;

  const auto result =
      ValidateMissionPreflight(profile, robot_navigation::ZoneCatalog(zones_path), config);

  EXPECT_FALSE(result.allowed);
  EXPECT_NE(result.message.find("battery insufficient"), std::string::npos);
}

TEST_F(MissionProfileTest, ValidateStationTransportPreflight_MissingRoute_ReturnsRejected) {
  const auto zones_path = test_dir_ / "zones.yaml";
  std::ofstream zones(zones_path);
  zones << "zones:\n";
  zones.close();

  StationCatalog catalog;
  catalog.stations.push_back(Station{"receiving", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(Station{"packing", "map", 1.0, 0.0, 0.0});
  MissionProfile profile;
  profile.mission_id = "station_route_missing";
  profile.frame_id = "map";
  profile.waypoints.push_back(MissionWaypoint{0.0, 0.0, 0.0});
  profile.waypoints.push_back(MissionWaypoint{1.0, 0.0, 0.0});
  MissionPreflightConfig config;
  config.require_station_route = true;

  const auto result = ValidateStationTransportPreflight(
      profile, catalog, "receiving", "packing", robot_navigation::ZoneCatalog(zones_path), config);

  EXPECT_FALSE(result.allowed);
  EXPECT_NE(result.message.find("no enabled station route"), std::string::npos);
}

TEST(MissionRecoveryPolicyTest, DecideMissionRecovery_RetryAvailable_ReturnsRetry) {
  MissionRecoveryConfig config;
  config.enabled = true;
  config.policy = "retry_then_dock";
  config.retry_limit = 2;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "retry");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, DecideMissionRecovery_RetryLimitReached_ReturnsDock) {
  MissionRecoveryConfig config;
  config.enabled = true;
  config.policy = "retry_then_dock";
  config.retry_limit = 1;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "dock");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, DecideMissionRecovery_DockMissionFailed_ReturnsManual) {
  MissionRecoveryConfig config;
  config.enabled = true;
  config.policy = "retry_then_dock";
  config.retry_limit = 2;

  const auto decision = DecideMissionRecovery(config, 0, true, "dock_return");

  EXPECT_EQ(decision.action, "manual");
  EXPECT_FALSE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, DecideMissionRecovery_Disabled_ReturnsNone) {
  MissionRecoveryConfig config;
  config.enabled = false;
  config.policy = "retry_then_dock";
  config.retry_limit = 2;

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "none");
  EXPECT_FALSE(decision.recoverable);
}

TEST_F(MissionProfileTest, LoadStationCatalog_ValidCatalog_ReturnsStations) {
  const auto path = test_dir_ / "stations.yaml";
  std::ofstream out(path);
  out << "frame_id: map\nstations:\n"
      << "  - id: receiving\n    x: 0.2\n    y: 0.0\n    yaw: 0.0\n"
      << "  - id: storage_a\n    x: 0.8\n    y: 0.4\n    yaw: 1.57\n"
      << "  - id: packing\n    x: 0.0\n    y: -0.8\n    yaw: -1.57\n"
      << "edges:\n"
      << "  - from: receiving\n    to: storage_a\n    distance_m: 1.2\n"
      << "    bidirectional: true\n    enabled: true\n"
      << "  - from: storage_a\n    to: packing\n    distance_m: 1.6\n"
      << "    bidirectional: true\n    enabled: true\n";
  out.close();

  const auto catalog = LoadStationCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->stations.size(), 3U);
  ASSERT_EQ(catalog->edges.size(), 2U);
  EXPECT_EQ(catalog->stations[0].id, "receiving");
  EXPECT_EQ(catalog->stations[0].frame_id, "map");
  EXPECT_DOUBLE_EQ(catalog->stations[2].y, -0.8);
  const auto* packing = FindStation(*catalog, "packing");
  ASSERT_NE(packing, nullptr);
  EXPECT_DOUBLE_EQ(packing->yaw, -1.57);
  const auto route_distance = EstimateRouteDistance(*catalog, "receiving", "packing");
  ASSERT_TRUE(route_distance.has_value());
  EXPECT_DOUBLE_EQ(*route_distance, 2.8);
}

TEST_F(MissionProfileTest, LoadStationCatalog_DuplicateStation_ReturnsNullopt) {
  const auto path = test_dir_ / "stations.yaml";
  std::ofstream out(path);
  out << "frame_id: map\nstations:\n"
      << "  - id: receiving\n    x: 0.2\n    y: 0.0\n"
      << "  - id: receiving\n    x: 0.4\n    y: 0.0\n";
  out.close();

  EXPECT_FALSE(LoadStationCatalog(path).has_value());
}

TEST_F(MissionProfileTest, LoadStationCatalog_UnknownRouteEndpoint_ReturnsNullopt) {
  const auto path = test_dir_ / "stations.yaml";
  std::ofstream out(path);
  out << "frame_id: map\nstations:\n"
      << "  - id: receiving\n    x: 0.2\n    y: 0.0\n"
      << "edges:\n"
      << "  - from: receiving\n    to: missing\n    distance_m: 1.0\n";
  out.close();

  EXPECT_FALSE(LoadStationCatalog(path).has_value());
}

TEST_F(MissionProfileTest, LoadFacilityCatalog_ValidCatalog_ReturnsResources) {
  const auto path = test_dir_ / "facilities.yaml";
  std::ofstream out(path);
  out << "resources:\n"
      << "  - id: charger_main\n"
      << "    type: charger\n"
      << "    station_id: dock\n"
      << "    enabled: true\n"
      << "    exclusive: true\n"
      << "  - id: receiving_door\n"
      << "    type: door\n"
      << "    station_id: receiving\n";
  out.close();

  const auto catalog = LoadFacilityCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->resources.size(), 2U);
  EXPECT_EQ(catalog->resources[0].id, "charger_main");
  EXPECT_EQ(catalog->resources[0].type, "charger");
  EXPECT_EQ(catalog->resources[0].station_id, "dock");
  const auto* door = FindFacilityResource(*catalog, "receiving_door");
  ASSERT_NE(door, nullptr);
  EXPECT_TRUE(door->enabled);
}

TEST_F(MissionProfileTest, LoadDockCatalog_ValidCatalog_ReturnsDock) {
  const auto path = test_dir_ / "docks.yaml";
  std::ofstream out(path);
  out << "docks:\n"
      << "  - id: main_dock\n"
      << "    station_id: dock\n"
      << "    approach_station_id: dock_entry\n"
      << "    charger_resource_id: charger_main\n"
      << "    enabled: true\n";
  out.close();

  const auto catalog = LoadDockCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->docks.size(), 1U);
  EXPECT_EQ(catalog->docks[0].id, "main_dock");
  EXPECT_EQ(catalog->docks[0].approach_station_id, "dock_entry");
  EXPECT_EQ(catalog->docks[0].charger_resource_id, "charger_main");
}

TEST_F(MissionProfileTest, LoadDockCatalog_DuplicateDock_ReturnsNullopt) {
  const auto path = test_dir_ / "docks.yaml";
  std::ofstream out(path);
  out << "docks:\n"
      << "  - id: main_dock\n"
      << "    station_id: dock\n"
      << "    charger_resource_id: charger_main\n"
      << "  - id: main_dock\n"
      << "    station_id: dock\n"
      << "    charger_resource_id: charger_backup\n";
  out.close();

  EXPECT_FALSE(LoadDockCatalog(path).has_value());
}

TEST_F(MissionProfileTest, LoadFacilityCatalog_DuplicateResource_ReturnsNullopt) {
  const auto path = test_dir_ / "facilities.yaml";
  std::ofstream out(path);
  out << "resources:\n"
      << "  - id: charger_main\n"
      << "    type: charger\n"
      << "  - id: charger_main\n"
      << "    type: charger\n";
  out.close();

  EXPECT_FALSE(LoadFacilityCatalog(path).has_value());
}

TEST_F(MissionProfileTest, FindFirstAvailableResource_SkipsReservedAndDisabled_ReturnsNext) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"charger_a", "charger", "dock", true, true});
  catalog.resources.push_back(FacilityResource{"charger_b", "charger", "dock", false, true});
  catalog.resources.push_back(FacilityResource{"charger_c", "charger", "dock", true, true});

  const auto* resource = FindFirstAvailableResource(catalog, "charger", {"charger_a"});

  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->id, "charger_c");
}

TEST_F(MissionProfileTest, FindFirstAvailableResource_SelectsDoorResource_ReturnsDoor) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"receiving_door", "door", "receiving", true, true});
  catalog.resources.push_back(FacilityResource{"freight_elevator", "elevator", "packing", true, true});

  const auto* resource = FindFirstAvailableResource(catalog, "door", {});

  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->id, "receiving_door");
  EXPECT_EQ(resource->station_id, "receiving");
}

TEST_F(MissionProfileTest, LoadFleetCatalog_ValidCatalog_ReturnsRobots) {
  const auto path = test_dir_ / "fleet.yaml";
  std::ofstream out(path);
  out << "robots:\n"
      << "  - id: robot_1\n"
      << "    enabled: true\n"
      << "    state: available\n"
      << "    battery_voltage: 24.5\n"
      << "    max_payload_kg: 40.0\n"
      << "    current_station_id: dock\n"
      << "    capabilities: transport,station_transport,warehouse\n"
      << "  - id: robot_2\n"
      << "    enabled: true\n"
      << "    state: charging\n"
      << "    battery_voltage: 23.0\n"
      << "    capabilities: transport\n";
  out.close();

  const auto catalog = LoadFleetCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->robots.size(), 2U);
  const auto* robot = FindFleetRobot(*catalog, "robot_1");
  ASSERT_NE(robot, nullptr);
  EXPECT_TRUE(RobotHasCapability(*robot, "station_transport"));
  EXPECT_EQ(robot->current_station_id, "dock");
}

TEST_F(MissionProfileTest, LoadFleetCatalog_DuplicateRobot_ReturnsNullopt) {
  const auto path = test_dir_ / "fleet.yaml";
  std::ofstream out(path);
  out << "robots:\n"
      << "  - id: robot_1\n"
      << "  - id: robot_1\n";
  out.close();

  EXPECT_FALSE(LoadFleetCatalog(path).has_value());
}

TEST(MissionFleetCatalogTest, SelectFleetRobot_SkipsBusyAndPicksHighestBattery) {
  FleetCatalog catalog;
  catalog.robots.push_back(
      FleetRobot{"robot_a", true, "busy", 25.0, 20.0, "dock", {"station_transport"}});
  catalog.robots.push_back(
      FleetRobot{"robot_b", true, "available", 23.0, 20.0, "dock", {"station_transport"}});
  catalog.robots.push_back(
      FleetRobot{"robot_c", true, "available", 24.0, 20.0, "dock", {"station_transport"}});

  const auto* selected = SelectFleetRobot(catalog, "station_transport", {});

  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->id, "robot_c");
}

TEST(MissionFleetCatalogTest, SelectFleetRobotForStationTask_PicksLowestRouteCost) {
  FleetCatalog fleet;
  fleet.robots.push_back(
      FleetRobot{"robot_a", true, "available", 24.8, 20.0, "dock", {"station_transport"}});
  fleet.robots.push_back(FleetRobot{
      "robot_b", true, "available", 23.5, 20.0, "receiving", {"station_transport"}});
  StationCatalog stations;
  stations.stations.push_back(Station{"dock", "map", -1.0, -1.0, 0.0});
  stations.stations.push_back(Station{"receiving", "map", 0.0, 0.0, 0.0});
  stations.stations.push_back(Station{"storage_a", "map", 1.0, 0.0, 0.0});
  stations.edges.push_back(StationEdge{"dock", "receiving", 3.0, true, true});
  stations.edges.push_back(StationEdge{"receiving", "storage_a", 1.0, true, true});
  FleetRobotTaskScore score;

  const auto* selected = SelectFleetRobotForStationTask(
      fleet, stations, "station_transport", "receiving", "storage_a", {}, &score);

  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->id, "robot_b");
  EXPECT_EQ(score.robot_id, "robot_b");
  EXPECT_DOUBLE_EQ(score.approach_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(score.task_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(score.total_distance_m, 1.0);
}

TEST(MissionFleetCatalogTest, SelectFleetRobotForStationTask_TieBreaksByBattery) {
  FleetCatalog fleet;
  fleet.robots.push_back(
      FleetRobot{"robot_a", true, "available", 24.1, 20.0, "dock", {"station_transport"}});
  fleet.robots.push_back(
      FleetRobot{"robot_b", true, "available", 24.7, 20.0, "dock", {"station_transport"}});
  StationCatalog stations;
  stations.stations.push_back(Station{"dock", "map", 0.0, 0.0, 0.0});
  stations.stations.push_back(Station{"receiving", "map", 1.0, 0.0, 0.0});
  stations.stations.push_back(Station{"storage_a", "map", 2.0, 0.0, 0.0});
  stations.edges.push_back(StationEdge{"dock", "receiving", 1.0, true, true});
  stations.edges.push_back(StationEdge{"receiving", "storage_a", 1.0, true, true});

  const auto* selected = SelectFleetRobotForStationTask(
      fleet, stations, "station_transport", "receiving", "storage_a", {}, nullptr);

  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->id, "robot_b");
}

TEST_F(MissionProfileTest, LoadScenarioCatalog_ValidCatalog_ReturnsTasks) {
  const auto path = test_dir_ / "scenarios.yaml";
  std::ofstream out(path);
  out << "tasks:\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: inbound\n"
      << "    label: inbound task\n"
      << "    pickup_station_id: receiving\n"
      << "    dropoff_station_id: storage_a\n"
      << "    required_capability: station_transport\n"
      << "    default_priority: 45\n";
  out.close();

  const auto catalog = LoadScenarioCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  ASSERT_EQ(catalog->tasks.size(), 1U);
  const auto* task = FindScenarioTask(*catalog, "warehouse", "inbound");
  ASSERT_NE(task, nullptr);
  EXPECT_EQ(task->label, "inbound task");
  EXPECT_EQ(task->pickup_station_id, "receiving");
  EXPECT_EQ(task->dropoff_station_id, "storage_a");
  EXPECT_EQ(task->required_capability, "station_transport");
  EXPECT_EQ(task->default_priority, 45);
}

TEST_F(MissionProfileTest, LoadScenarioCatalog_ValidWorkflow_ReturnsWorkflow) {
  const auto path = test_dir_ / "scenarios.yaml";
  std::ofstream out(path);
  out << "tasks:\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: inbound\n"
      << "    pickup_station_id: receiving\n"
      << "    dropoff_station_id: storage_a\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: outbound\n"
      << "    pickup_station_id: storage_a\n"
      << "    dropoff_station_id: packing\n"
      << "workflows:\n"
      << "  - scenario_id: warehouse\n"
      << "    workflow_id: crossdock\n"
      << "    label: crossdock flow\n"
      << "    task_ids: inbound,outbound\n"
      << "    default_priority: 55\n";
  out.close();

  const auto catalog = LoadScenarioCatalog(path);

  ASSERT_TRUE(catalog.has_value());
  const auto* workflow = FindScenarioWorkflow(*catalog, "warehouse", "crossdock");
  ASSERT_NE(workflow, nullptr);
  EXPECT_EQ(workflow->label, "crossdock flow");
  ASSERT_EQ(workflow->task_ids.size(), 2U);
  EXPECT_EQ(workflow->task_ids[0], "inbound");
  EXPECT_EQ(workflow->task_ids[1], "outbound");
  EXPECT_EQ(workflow->default_priority, 55);
}

TEST_F(MissionProfileTest, LoadScenarioCatalog_DuplicateTask_ReturnsNullopt) {
  const auto path = test_dir_ / "scenarios.yaml";
  std::ofstream out(path);
  out << "tasks:\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: inbound\n"
      << "    pickup_station_id: receiving\n"
      << "    dropoff_station_id: storage_a\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: inbound\n"
      << "    pickup_station_id: receiving\n"
      << "    dropoff_station_id: storage_a\n";
  out.close();

  EXPECT_FALSE(LoadScenarioCatalog(path).has_value());
}

TEST_F(MissionProfileTest, LoadScenarioCatalog_WorkflowUnknownTask_ReturnsNullopt) {
  const auto path = test_dir_ / "scenarios.yaml";
  std::ofstream out(path);
  out << "tasks:\n"
      << "  - scenario_id: warehouse\n"
      << "    task_id: inbound\n"
      << "    pickup_station_id: receiving\n"
      << "    dropoff_station_id: storage_a\n"
      << "workflows:\n"
      << "  - scenario_id: warehouse\n"
      << "    workflow_id: broken\n"
      << "    task_ids: inbound,missing\n";
  out.close();

  EXPECT_FALSE(LoadScenarioCatalog(path).has_value());
}

TEST_F(MissionProfileTest, FindScenarioTask_MissingTask_ReturnsNull) {
  ScenarioCatalog catalog;
  ScenarioTask task;
  task.scenario_id = "warehouse";
  task.task_id = "inbound";
  task.pickup_station_id = "receiving";
  task.dropoff_station_id = "storage_a";
  task.required_capability = "station_transport";
  task.default_priority = 45;
  catalog.tasks.push_back(task);

  EXPECT_EQ(FindScenarioTask(catalog, "warehouse", "missing"), nullptr);
}

}  // namespace robot_tasks
