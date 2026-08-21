#include "robot_tasks/docking_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(DockingBehaviorTreeTest, SuccessfulContact_CompletesDockingThroughBtAction) {
  DockingBehaviorTreeInput input;
  input.dock_state = DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0};
  input.resource_id = "charger_a";

  const auto result = TickDockingBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "complete_docking");
  EXPECT_EQ(result.transition.event_state, "CHARGING");
  EXPECT_EQ(result.dock_state.state, "CHARGING");
  EXPECT_EQ(result.reservation.status, "CHARGING");
}

TEST(DockingBehaviorTreeTest, FirstContactFailure_StagesRetryThroughBtBranch) {
  DockingBehaviorTreeInput input;
  input.dock_state = DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", false, 0};
  input.resource_id = "charger_a";

  const auto result = TickDockingBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "contact_retry");
  EXPECT_EQ(result.transition.event_state, "DOCKING_CONTACT_FAILED");
  EXPECT_EQ(result.dock_state.state, "FAILED");
  EXPECT_EQ(result.dock_state.contact_attempts, 1);
  EXPECT_TRUE(result.dock_state.simulate_contact_success);
  EXPECT_EQ(result.reservation.status, "DOCKING_CONTACT_FAILED");
}

}  // namespace robot_tasks
