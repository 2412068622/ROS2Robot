#include "robot_tasks/mission_charging_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

OpportunityChargingPlanRequest MakeOpportunityRequest() {
  OpportunityChargingPlanRequest request;
  request.current_battery_voltage = 23.0;
  request.default_opportunity_threshold_voltage = 24.0;
  request.default_critical_threshold_voltage = 21.0;
  request.default_priority = 40;
  request.queue_empty = true;
  return request;
}

}  // namespace

TEST(MissionChargingWorkflowTest, PlanOpportunityCharging_NoBattery_RejectsRequest) {
  auto request = MakeOpportunityRequest();
  request.current_battery_voltage = 0.0;

  const auto decision = PlanOpportunityCharging(request);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.queue_charging);
  EXPECT_EQ(decision.branch, "battery_unavailable");
  EXPECT_EQ(decision.message, "battery voltage unavailable for opportunity charging");
}

TEST(MissionChargingWorkflowTest, PlanOpportunityCharging_AboveThreshold_SkipsWithoutQueueing) {
  auto request = MakeOpportunityRequest();
  request.requested_battery_voltage = 25.0;

  const auto decision = PlanOpportunityCharging(request);

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.queue_charging);
  EXPECT_EQ(decision.branch, "above_threshold");
  EXPECT_EQ(decision.evaluated_battery_voltage, 25.0);
}

TEST(MissionChargingWorkflowTest, PlanOpportunityCharging_BusyNonCriticalWithIdleRequired_Skips) {
  auto request = MakeOpportunityRequest();
  request.requested_require_idle_queue = true;
  request.mission_active = true;

  const auto decision = PlanOpportunityCharging(request);

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.queue_charging);
  EXPECT_EQ(decision.branch, "busy_noncritical");
}

TEST(MissionChargingWorkflowTest, PlanOpportunityCharging_CriticalBusyMission_QueuesAndPreempts) {
  auto request = MakeOpportunityRequest();
  request.requested_battery_voltage = 20.5;
  request.default_require_idle_queue = true;
  request.mission_active = true;

  const auto decision = PlanOpportunityCharging(request);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.queue_charging);
  EXPECT_TRUE(decision.critical_battery);
  EXPECT_TRUE(decision.preempt_current);
  EXPECT_EQ(decision.priority, 40);
  EXPECT_EQ(decision.queued_message_prefix, "critical ");
}

TEST(MissionChargingWorkflowTest, PlanOpportunityCharging_RequestedPriorityAndPreempt_ArePreserved) {
  auto request = MakeOpportunityRequest();
  request.requested_priority = 80;
  request.requested_preempt_current = true;

  const auto decision = PlanOpportunityCharging(request);

  EXPECT_TRUE(decision.queue_charging);
  EXPECT_FALSE(decision.critical_battery);
  EXPECT_TRUE(decision.preempt_current);
  EXPECT_EQ(decision.priority, 80);
  EXPECT_EQ(decision.queued_message_prefix, "opportunity ");
}

TEST(MissionChargingWorkflowTest, PlanLowBatteryDock_Disabled_PreservesPreviousRequestFlag) {
  LowBatteryDockRequest request;
  request.enabled = false;
  request.battery_voltage = 20.0;
  request.threshold_voltage = 21.0;
  request.request_already_sent = true;

  const auto decision = PlanLowBatteryDock(request);

  EXPECT_FALSE(decision.request_dock_return);
  EXPECT_TRUE(decision.low_battery_request_sent);
  EXPECT_EQ(decision.branch, "disabled");
}

TEST(MissionChargingWorkflowTest, PlanLowBatteryDock_AboveThreshold_ResetsRequestFlag) {
  LowBatteryDockRequest request;
  request.enabled = true;
  request.battery_voltage = 23.0;
  request.threshold_voltage = 21.0;
  request.request_already_sent = true;

  const auto decision = PlanLowBatteryDock(request);

  EXPECT_FALSE(decision.request_dock_return);
  EXPECT_FALSE(decision.low_battery_request_sent);
  EXPECT_EQ(decision.branch, "above_threshold");
}

TEST(MissionChargingWorkflowTest, PlanLowBatteryDock_BelowThreshold_RequestsDockOnce) {
  LowBatteryDockRequest request;
  request.enabled = true;
  request.battery_voltage = 20.5;
  request.threshold_voltage = 21.0;
  request.request_already_sent = false;

  const auto decision = PlanLowBatteryDock(request);

  EXPECT_TRUE(decision.request_dock_return);
  EXPECT_TRUE(decision.low_battery_request_sent);
  EXPECT_EQ(decision.branch, "request_dock_return");
  EXPECT_EQ(decision.reason, "low battery 20.500000V");
}

TEST(MissionChargingWorkflowTest, PlanLowBatteryDock_AlreadyRequested_DoesNotRepeat) {
  LowBatteryDockRequest request;
  request.enabled = true;
  request.battery_voltage = 20.5;
  request.threshold_voltage = 21.0;
  request.request_already_sent = true;

  const auto decision = PlanLowBatteryDock(request);

  EXPECT_FALSE(decision.request_dock_return);
  EXPECT_TRUE(decision.low_battery_request_sent);
  EXPECT_EQ(decision.branch, "already_requested");
}

TEST(MissionChargingWorkflowTest, PlanChargingRequestGate_MissingCatalogRejects) {
  ChargingRequestGateRequest request;
  request.charger_id = "charger_a";
  request.station_catalog_loaded = false;
  request.station_catalog_error_message = "failed to load station catalog: stations.yaml";

  const auto decision = PlanChargingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.charger_id, "charger_a");
  EXPECT_EQ(decision.message, "failed to load station catalog: stations.yaml");
}

TEST(MissionChargingWorkflowTest, PlanChargingRequestGate_MissingChargerStationRejects) {
  ChargingRequestGateRequest request;
  request.charger_id = "charger_a";
  request.charger_station_id = "dock_station";
  request.charger_station_known = false;

  const auto decision = PlanChargingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "charger station not found: dock_station");
}

TEST(MissionChargingWorkflowTest, PlanChargingRequestGate_DuplicateRejects) {
  ChargingRequestGateRequest request;
  request.mission_id = "charging_request_alpha";
  request.charger_id = "charger_a";
  request.mission_already_active_or_queued = true;

  const auto decision = PlanChargingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "charging_request_alpha");
  EXPECT_EQ(decision.charger_id, "charger_a");
  EXPECT_EQ(
      decision.message,
      "charging mission already active or queued: charging_request_alpha");
}

TEST(MissionChargingWorkflowTest, PlanChargingRequestGate_PreflightRejects) {
  ChargingRequestGateRequest request;
  request.mission_id = "charging_request_alpha";
  request.charger_id = "charger_a";
  request.preflight.allowed = false;
  request.preflight.message = "battery below minimum";

  const auto decision = PlanChargingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "charging mission preflight rejected: battery below minimum");
}

TEST(MissionChargingWorkflowTest, PlanChargingRequestGate_ValidRequestAccepted) {
  ChargingRequestGateRequest request;
  request.mission_id = "charging_request_alpha";
  request.charger_id = "charger_a";

  const auto decision = PlanChargingRequestGate(request);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "charging_request_alpha");
  EXPECT_EQ(decision.charger_id, "charger_a");
  EXPECT_TRUE(decision.message.empty());
}

}  // namespace robot_tasks
