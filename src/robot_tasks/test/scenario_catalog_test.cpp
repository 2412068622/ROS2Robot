#include "robot_tasks/scenario_catalog.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

ScenarioCatalog Catalog() {
  ScenarioCatalog catalog;
  catalog.tasks.push_back(ScenarioTask{
      "warehouse", "pickup", "", "station_transport", "receiving", "packing"});
  catalog.tasks.push_back(ScenarioTask{
      "warehouse", "dropoff", "", "station_transport", "packing", "storage"});
  catalog.tasks.push_back(ScenarioTask{
      "warehouse",
      "door_pass",
      "",
      "facility_action",
      "",
      "",
      {},
      "station_transport",
      "door_1",
      "door",
      "open",
      true,
      35});
  catalog.tasks.push_back(ScenarioTask{
      "warehouse",
      "patrol",
      "",
      "station_sequence",
      "",
      "",
      {"receiving", "packing", "storage"},
      "station_transport",
      "",
      "",
      "",
      false,
      32});
  catalog.workflows.push_back(
      ScenarioWorkflow{"warehouse", "crossdock", "Crossdock", {"pickup", "dropoff"}, 40});
  return catalog;
}

TEST(ScenarioCatalogTest, PlanScenarioWorkflowSubmit_ValidBuildsStepPlans) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioWorkflowSubmit(
      ScenarioWorkflowSubmitRequest{"warehouse", "crossdock", "order_a", 50, true, true},
      &catalog,
      "");

  ASSERT_TRUE(decision.success);
  ASSERT_EQ(decision.steps.size(), 2U);
  EXPECT_EQ(decision.steps[0].scenario_id, "warehouse");
  EXPECT_EQ(decision.steps[0].task_id, "pickup");
  EXPECT_EQ(decision.steps[0].order_id, "order_a_step_1");
  EXPECT_EQ(decision.steps[0].priority, 50);
  EXPECT_TRUE(decision.steps[0].start_if_idle);
  EXPECT_TRUE(decision.steps[0].preempt_current);
  EXPECT_EQ(decision.steps[1].task_id, "dropoff");
  EXPECT_EQ(decision.steps[1].order_id, "order_a_step_2");
  EXPECT_EQ(decision.steps[1].priority, 49);
  EXPECT_FALSE(decision.steps[1].start_if_idle);
  EXPECT_FALSE(decision.steps[1].preempt_current);
}

TEST(ScenarioCatalogTest, PlanScenarioWorkflowSubmit_DefaultOrderAndPriority) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioWorkflowSubmit(
      ScenarioWorkflowSubmitRequest{"warehouse", "crossdock", "", 0, false, false},
      &catalog,
      "");

  ASSERT_TRUE(decision.success);
  ASSERT_FALSE(decision.steps.empty());
  EXPECT_EQ(decision.steps[0].order_id, "workflow_warehouse_crossdock_step_1");
  EXPECT_EQ(decision.steps[0].priority, 40);
  EXPECT_FALSE(decision.steps[0].start_if_idle);
}

TEST(ScenarioCatalogTest, PlanScenarioWorkflowSubmit_MissingCatalogRejects) {
  const auto decision = PlanScenarioWorkflowSubmit(
      ScenarioWorkflowSubmitRequest{"warehouse", "crossdock", "", 0, true, false},
      nullptr,
      "failed to load scenario catalog: missing.yaml");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "failed to load scenario catalog: missing.yaml");
}

TEST(ScenarioCatalogTest, PlanScenarioWorkflowSubmit_UnknownWorkflowRejects) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioWorkflowSubmit(
      ScenarioWorkflowSubmitRequest{"warehouse", "missing", "", 0, true, false},
      &catalog,
      "");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "unknown scenario workflow: warehouse/missing");
}

TEST(ScenarioCatalogTest, PlanScenarioTaskSubmit_FleetStationTaskBuildsDispatchFields) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioTaskSubmit(
      ScenarioTaskSubmitRequest{"warehouse", "pickup", "order_a", 50, true, true},
      &catalog,
      "");

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.dispatch_kind, ScenarioTaskDispatchKind::kFleetStation);
  EXPECT_EQ(decision.order_id, "order_a");
  EXPECT_EQ(decision.priority, 50);
  EXPECT_EQ(decision.pickup_station, "receiving");
  EXPECT_EQ(decision.dropoff_station, "packing");
  EXPECT_TRUE(decision.start_if_idle);
  EXPECT_TRUE(decision.preempt_current);
}

TEST(ScenarioCatalogTest, PlanScenarioTaskSubmit_FacilityTaskBuildsDispatchFields) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioTaskSubmit(
      ScenarioTaskSubmitRequest{"warehouse", "door_pass", "", 0, true, false},
      &catalog,
      "");

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.dispatch_kind, ScenarioTaskDispatchKind::kFacilityAction);
  EXPECT_EQ(decision.order_id, "scenario_warehouse_door_pass");
  EXPECT_EQ(decision.priority, 35);
  EXPECT_EQ(decision.resource_id, "door_1");
  EXPECT_EQ(decision.resource_type, "door");
  EXPECT_EQ(decision.action, "open");
  EXPECT_TRUE(decision.hold_after_action);
}

TEST(ScenarioCatalogTest, PlanScenarioTaskSubmit_StationSequenceTaskBuildsDispatchFields) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioTaskSubmit(
      ScenarioTaskSubmitRequest{"warehouse", "patrol", "", 0, false, false},
      &catalog,
      "");

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.dispatch_kind, ScenarioTaskDispatchKind::kStationSequence);
  EXPECT_EQ(decision.order_id, "scenario_warehouse_patrol");
  EXPECT_EQ(decision.priority, 32);
  EXPECT_EQ(
      decision.station_ids,
      (std::vector<std::string>{"receiving", "packing", "storage"}));
  EXPECT_FALSE(decision.start_if_idle);
}

TEST(ScenarioCatalogTest, PlanScenarioTaskSubmit_MissingCatalogRejects) {
  const auto decision = PlanScenarioTaskSubmit(
      ScenarioTaskSubmitRequest{"warehouse", "pickup", "", 0, true, false},
      nullptr,
      "failed to load scenario catalog: missing.yaml");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "failed to load scenario catalog: missing.yaml");
}

TEST(ScenarioCatalogTest, PlanScenarioTaskSubmit_UnknownTaskRejects) {
  const auto catalog = Catalog();

  const auto decision = PlanScenarioTaskSubmit(
      ScenarioTaskSubmitRequest{"warehouse", "missing", "", 0, true, false},
      &catalog,
      "");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "unknown scenario task: warehouse/missing");
}

}  // namespace
}  // namespace robot_tasks
