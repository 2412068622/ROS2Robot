#include "robot_tasks/business_order_catalog.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

BusinessOrderCatalog Catalog() {
  BusinessOrderCatalog catalog;
  catalog.order_types.push_back(
      BusinessOrderType{"lab_delivery", "Lab Delivery", "laboratory", "sample_route"});
  return catalog;
}

TEST(BusinessOrderCatalogTest, PlanBusinessOrderSubmit_ValidBuildsWorkflowRoute) {
  const auto catalog = Catalog();

  const auto decision = PlanBusinessOrderSubmit(
      BusinessOrderSubmitRequest{"order_a", "lab_delivery", 20, true, false},
      &catalog,
      "");

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.business_order_id, "order_a");
  EXPECT_EQ(decision.business_type, "lab_delivery");
  EXPECT_EQ(decision.routed_service, "/submit_scenario_workflow");
  EXPECT_EQ(decision.routed_scenario_id, "laboratory");
  EXPECT_EQ(decision.routed_workflow_id, "sample_route");
  EXPECT_EQ(decision.workflow_order_id, "business_order_a");
  EXPECT_EQ(decision.priority, 20);
  EXPECT_TRUE(decision.start_if_idle);
  EXPECT_FALSE(decision.preempt_current);
  EXPECT_EQ(decision.message, "accepted business order order_a as laboratory/sample_route");
}

TEST(BusinessOrderCatalogTest, PlanBusinessOrderSubmit_EmptyInputRejectsBeforeCatalog) {
  const auto decision = PlanBusinessOrderSubmit(
      BusinessOrderSubmitRequest{"", "lab_delivery", 0, true, false},
      nullptr,
      "catalog should not be required");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "business_order_id or business_type is empty");
}

TEST(BusinessOrderCatalogTest, PlanBusinessOrderSubmit_MissingCatalogRejectsWithLoadMessage) {
  const auto decision = PlanBusinessOrderSubmit(
      BusinessOrderSubmitRequest{"order_a", "lab_delivery", 0, true, false},
      nullptr,
      "failed to load business order catalog: missing.yaml");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "failed to load business order catalog: missing.yaml");
}

TEST(BusinessOrderCatalogTest, PlanBusinessOrderSubmit_UnsupportedTypeRejects) {
  const auto catalog = Catalog();

  const auto decision = PlanBusinessOrderSubmit(
      BusinessOrderSubmitRequest{"order_a", "unknown", 0, true, false},
      &catalog,
      "");

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "unsupported business_type: unknown");
}

}  // namespace
}  // namespace robot_tasks
