#include "robot_tasks/operator_snapshot.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(OperatorSnapshotTest, BuildOperatorSnapshotProjection_CombinesRuntimeViews) {
  MissionProfile profile;
  profile.mission_id = "mission_a";
  const std::vector<QueuedMission> queue{
      QueuedMission{profile, "mission.yaml", 30, 1U},
  };

  FleetCatalog fleet;
  fleet.robots.push_back(
      FleetRobot{"robot_1", true, "available", 24.5, 40.0, "dock", {"station_transport"}});

  RemoteFleetTaskMap remote_tasks;
  remote_tasks["task_a"] =
      RemoteFleetTaskState{"task_a", "robot_2", "receiving", "storage", "DISPATCHED_REMOTE", ""};

  FacilityCatalog facilities;
  facilities.resources.push_back(FacilityResource{"door_1", "door", "receiving", true, true});
  FacilityReservationMap reservations;
  reservations["door_1"] = ResourceReservation{"holder_a", "mission_a", "occupied", true};

  BusinessOrderCatalog business_orders;
  business_orders.order_types.push_back(
      BusinessOrderType{"lab_delivery", "Lab Delivery", "laboratory", "sample_route"});

  const std::vector<MissionEvent> events{
      MakeMissionEvent("t1", "mission_a", "QUEUED", "READY", "queued", true),
      MakeMissionEvent("t2", "mission_a", "FINISHED", "RUNNING", "done", true),
  };

  OperatorSnapshotRequest request;
  request.mission_state = "READY";
  request.previous_mission_state = "BOOT";
  request.active_mission_id = "active";
  request.mission_message = "ready";
  request.recoverable = true;
  request.mission_active = false;
  request.paused = false;
  request.mission_queue = queue;
  request.paused_workflow_order_ids = {"wf_b", "wf_a"};
  request.fleet = &fleet;
  request.remote_tasks = &remote_tasks;
  request.facilities = &facilities;
  request.facility_reservations = &reservations;
  request.business_orders = &business_orders;
  request.mission_events = events;
  request.event_limit = 1;

  const auto projection = BuildOperatorSnapshotProjection(request);

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.mission_state, "READY");
  EXPECT_EQ(projection.queue_size, 1);
  EXPECT_EQ(projection.queued_mission_ids, (std::vector<std::string>{"mission_a"}));
  EXPECT_EQ(projection.queued_priorities, (std::vector<int>{30}));
  EXPECT_EQ(projection.paused_workflow_order_ids, (std::vector<std::string>{"wf_a", "wf_b"}));
  EXPECT_EQ(projection.fleet_robot_ids, (std::vector<std::string>{"robot_1"}));
  EXPECT_EQ(projection.fleet_capabilities, (std::vector<std::string>{"station_transport"}));
  EXPECT_EQ(projection.remote_task_robot_ids, (std::vector<std::string>{"robot_2"}));
  EXPECT_EQ(projection.facility_resource_ids, (std::vector<std::string>{"door_1"}));
  EXPECT_EQ(projection.facility_available, (std::vector<bool>{false}));
  EXPECT_EQ(projection.facility_status, (std::vector<std::string>{"occupied"}));
  EXPECT_EQ(projection.business_types, (std::vector<std::string>{"lab_delivery"}));
  EXPECT_EQ(projection.recent_event_states, (std::vector<std::string>{"FINISHED"}));
  EXPECT_EQ(projection.message, "operator snapshot queue_size=1 robots=1 resources=1 events=1");
}

}  // namespace
}  // namespace robot_tasks
