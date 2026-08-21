#include "robot_tasks/fleet_runtime_state.hpp"

#include <gtest/gtest.h>

namespace {

robot_tasks::FleetCatalog Fleet() {
  robot_tasks::FleetCatalog catalog;
  catalog.robots.push_back(robot_tasks::FleetRobot{
      "robot_1", true, "available", 24.5, 40.0, "dock", {"station_transport"}});
  catalog.robots.push_back(robot_tasks::FleetRobot{
      "robot_2", true, "charging", 23.0, 20.0, "dock", {"station_transport"}});
  return catalog;
}

robot_tasks::StationCatalog Stations() {
  robot_tasks::StationCatalog catalog;
  catalog.stations.push_back(robot_tasks::Station{"dock", "map", 0.0, 0.0, 0.0});
  catalog.stations.push_back(robot_tasks::Station{"receiving", "map", 1.0, 0.0, 0.0});
  return catalog;
}

}  // namespace

TEST(FleetRuntimeStateTest, ApplyRuntimeFleetState_OverlaysRequestedFieldsOnly) {
  auto catalog = Fleet();
  std::unordered_map<std::string, robot_tasks::RuntimeFleetRobotState> runtime;
  runtime["robot_2"].update_state = true;
  runtime["robot_2"].state = "available";
  runtime["robot_2"].update_battery_voltage = true;
  runtime["robot_2"].battery_voltage = 25.2;

  robot_tasks::ApplyRuntimeFleetState(catalog, runtime);

  const auto* robot = robot_tasks::FindFleetRobot(catalog, "robot_2");
  ASSERT_NE(robot, nullptr);
  EXPECT_TRUE(robot->enabled);
  EXPECT_EQ(robot->state, "available");
  EXPECT_DOUBLE_EQ(robot->battery_voltage, 25.2);
  EXPECT_EQ(robot->current_station_id, "dock");
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_ValidUpdate_ReturnsEffectiveRobot) {
  const auto stations = Stations();
  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "robot_2", true, "available", 25.2, "receiving", false, true, true, true},
      Fleet(), &stations, {});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_TRUE(decision.runtime_state.update_state);
  EXPECT_TRUE(decision.runtime_state.update_battery_voltage);
  EXPECT_TRUE(decision.runtime_state.update_current_station_id);
  EXPECT_EQ(decision.effective_robot.id, "robot_2");
  EXPECT_EQ(decision.effective_robot.state, "available");
  EXPECT_DOUBLE_EQ(decision.effective_robot.battery_voltage, 25.2);
  EXPECT_EQ(decision.effective_robot.current_station_id, "receiving");
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_MergesWithExistingRuntimeState) {
  std::unordered_map<std::string, robot_tasks::RuntimeFleetRobotState> runtime;
  runtime["robot_2"].update_battery_voltage = true;
  runtime["robot_2"].battery_voltage = 25.2;
  const auto stations = Stations();

  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "robot_2", true, "available", 0.0, "", false, true, false, false},
      Fleet(), &stations, runtime);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.effective_robot.state, "available");
  EXPECT_DOUBLE_EQ(decision.effective_robot.battery_voltage, 25.2);
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_UnknownRobotRejectsWithoutMutation) {
  const auto stations = Stations();
  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "missing", true, "available", 25.0, "dock", false, true, true, true},
      Fleet(), &stations, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.message, "unknown fleet robot: missing");
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_EmptyStateRejectsWhenRequested) {
  const auto stations = Stations();
  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "robot_2", true, "", 0.0, "", false, true, false, false},
      Fleet(), &stations, {});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "state is empty while update_state is true");
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_InvalidBatteryRejects) {
  const auto stations = Stations();
  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "robot_2", true, "", -1.0, "", false, false, true, false},
      Fleet(), &stations, {});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "battery_voltage must be finite and non-negative");
}

TEST(FleetRuntimeStateTest, PlanFleetRuntimeStateUpdate_UnknownStationRejects) {
  const auto stations = Stations();
  const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
      robot_tasks::FleetRuntimeUpdateRequest{
          "robot_2", true, "", 0.0, "missing_station", false, false, false, true},
      Fleet(), &stations, {});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "unknown current_station_id: missing_station");
}

TEST(FleetRuntimeStateTest, BuildFleetRobotListProjection_FiltersAndFormatsCapabilities) {
  auto catalog = Fleet();
  catalog.robots.push_back(robot_tasks::FleetRobot{
      "robot_3", true, "available", 24.0, 20.0, "dock", {"inspection", "station_transport"}});

  const auto projection = robot_tasks::BuildFleetRobotListProjection(
      catalog, robot_tasks::FleetRobotListRequest{false, "inspection"});

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.robot_ids, (std::vector<std::string>{"robot_3"}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{true}));
  EXPECT_EQ(projection.states, (std::vector<std::string>{"available"}));
  EXPECT_EQ(projection.battery_voltage, (std::vector<double>{24.0}));
  EXPECT_EQ(projection.current_station_ids, (std::vector<std::string>{"dock"}));
  EXPECT_EQ(projection.capabilities, (std::vector<std::string>{"inspection,station_transport"}));
  EXPECT_EQ(projection.message, "loaded 1 fleet robot(s)");
}

TEST(FleetRuntimeStateTest, BuildFleetRobotListProjection_SkipsDisabledUnlessRequested) {
  auto catalog = Fleet();
  catalog.robots[1].enabled = false;
  catalog.robots[1].capabilities = {"inspection"};

  const auto hidden = robot_tasks::BuildFleetRobotListProjection(
      catalog, robot_tasks::FleetRobotListRequest{false, "inspection"});
  EXPECT_TRUE(hidden.robot_ids.empty());

  const auto included = robot_tasks::BuildFleetRobotListProjection(
      catalog, robot_tasks::FleetRobotListRequest{true, "inspection"});
  EXPECT_EQ(included.robot_ids, (std::vector<std::string>{"robot_2"}));
  EXPECT_EQ(included.enabled, (std::vector<bool>{false}));
}

TEST(FleetRuntimeStateTest, PlanRemoteFleetTaskStateUpdate_EmptyTaskOrStateRejects) {
  const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
      robot_tasks::RemoteFleetTaskUpdateRequest{"", "DONE", ""}, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.response_message, "task_id or state is empty");
}

TEST(FleetRuntimeStateTest, PlanRemoteFleetTaskStateUpdate_UnknownTaskRejects) {
  const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
      robot_tasks::RemoteFleetTaskUpdateRequest{"task_a", "DONE", ""}, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.task.task_id, "task_a");
  EXPECT_EQ(decision.response_message, "unknown remote task: task_a");
}

TEST(FleetRuntimeStateTest, PlanRemoteFleetTaskStateUpdate_DefaultMessageUpdatesTaskAndEvent) {
  robot_tasks::RemoteFleetTaskMap remote_tasks;
  remote_tasks["task_a"] =
      robot_tasks::RemoteFleetTaskState{"task_a", "robot_2", "receiving", "packing",
                                        "DISPATCHED_REMOTE", "initial dispatch"};

  const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
      robot_tasks::RemoteFleetTaskUpdateRequest{"task_a", "COMPLETED", ""}, remote_tasks);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.task.state, "COMPLETED");
  EXPECT_EQ(decision.task.message, "remote task state updated");
  EXPECT_EQ(decision.previous_state, "DISPATCHED_REMOTE");
  EXPECT_EQ(decision.response_message, "updated remote task task_a to COMPLETED");
  EXPECT_EQ(decision.event_mission_id, "remote_fleet_task_task_a");
  EXPECT_EQ(decision.event_state, "COMPLETED");
  EXPECT_EQ(decision.event_previous_state, "DISPATCHED_REMOTE");
  EXPECT_EQ(decision.event_message, "remote task state updated");
  EXPECT_TRUE(decision.event_recoverable);
}

TEST(FleetRuntimeStateTest, PlanRemoteFleetTaskStateUpdate_ExplicitMessageIsPreserved) {
  robot_tasks::RemoteFleetTaskMap remote_tasks;
  remote_tasks["task_a"] =
      robot_tasks::RemoteFleetTaskState{"task_a", "robot_2", "receiving", "packing",
                                        "DISPATCHED_REMOTE", "initial dispatch"};

  const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
      robot_tasks::RemoteFleetTaskUpdateRequest{"task_a", "FAILED", "blocked at aisle"},
      remote_tasks);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.task.message, "blocked at aisle");
  EXPECT_EQ(decision.event_message, "blocked at aisle");
  EXPECT_EQ(decision.event_state, "FAILED");
}

TEST(FleetRuntimeStateTest, PlanFleetStationTaskDispatch_LocalRobotUsesStationOrder) {
  const auto decision = robot_tasks::PlanFleetStationTaskDispatch(
      robot_tasks::FleetStationTaskDispatchRequest{
          "task_a", "robot_1", "robot_1", "receiving", "packing", "READY", 4.2},
      {});

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.dispatch_remote);
  EXPECT_FALSE(decision.mutate_remote_tasks);
  EXPECT_EQ(decision.station_order_id, "fleet_task_a");
  EXPECT_TRUE(decision.mission_id.empty());
}

TEST(FleetRuntimeStateTest, PlanFleetStationTaskDispatch_RemoteRobotCreatesTaskAndEvent) {
  const auto decision = robot_tasks::PlanFleetStationTaskDispatch(
      robot_tasks::FleetStationTaskDispatchRequest{
          "task_a", "robot_2", "robot_1", "receiving", "packing", "READY", 4.2},
      {});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.dispatch_remote);
  EXPECT_TRUE(decision.mutate_remote_tasks);
  EXPECT_EQ(decision.remote_task.task_id, "task_a");
  EXPECT_EQ(decision.remote_task.assigned_robot_id, "robot_2");
  EXPECT_EQ(decision.remote_task.pickup_station, "receiving");
  EXPECT_EQ(decision.remote_task.dropoff_station, "packing");
  EXPECT_EQ(decision.remote_task.state, "DISPATCHED_REMOTE");
  EXPECT_EQ(decision.mission_id, "remote_fleet_task_task_a");
  EXPECT_EQ(decision.event_mission_id, "remote_fleet_task_task_a");
  EXPECT_EQ(decision.event_state, "DISPATCHED_REMOTE");
  EXPECT_EQ(decision.event_previous_state, "READY");
  EXPECT_EQ(decision.event_message, "dispatched remote fleet task task_a to robot_2");
  EXPECT_EQ(
      decision.response_message,
      "dispatched remote fleet task task_a to robot_2 estimated_total_distance_m=4.200000");
}

TEST(FleetRuntimeStateTest, PlanFleetStationTaskDispatch_DuplicateRemoteTaskRejects) {
  robot_tasks::RemoteFleetTaskMap remote_tasks;
  remote_tasks["task_a"] =
      robot_tasks::RemoteFleetTaskState{"task_a", "robot_2", "receiving", "packing",
                                        "DISPATCHED_REMOTE", "initial dispatch"};

  const auto decision = robot_tasks::PlanFleetStationTaskDispatch(
      robot_tasks::FleetStationTaskDispatchRequest{
          "task_a", "robot_2", "robot_1", "receiving", "packing", "READY", 4.2},
      remote_tasks);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.dispatch_remote);
  EXPECT_FALSE(decision.mutate_remote_tasks);
  EXPECT_EQ(decision.response_message, "remote fleet task already exists: task_a");
}

TEST(FleetRuntimeStateTest, PlanFleetStationTaskDispatch_EmptyTaskOrRobotRejects) {
  const auto empty_task = robot_tasks::PlanFleetStationTaskDispatch(
      robot_tasks::FleetStationTaskDispatchRequest{
          "", "robot_2", "robot_1", "receiving", "packing", "READY", 4.2},
      {});
  EXPECT_FALSE(empty_task.success);
  EXPECT_EQ(empty_task.response_message, "fleet task_id is empty");

  const auto empty_robot = robot_tasks::PlanFleetStationTaskDispatch(
      robot_tasks::FleetStationTaskDispatchRequest{
          "task_a", "", "robot_1", "receiving", "packing", "READY", 4.2},
      {});
  EXPECT_FALSE(empty_robot.success);
  EXPECT_EQ(empty_robot.response_message, "assigned robot id is empty");
}
