#include "robot_tasks/payload_workflow.hpp"

#include <gtest/gtest.h>

namespace {

robot_tasks::PayloadWorkflowState EmptyState() {
  robot_tasks::PayloadWorkflowState state;
  state.capacity_kg = 30.0;
  state.top_module_type = "manual_load";
  return state;
}

robot_tasks::PayloadWorkflowState LoadedState() {
  auto state = EmptyState();
  state.loaded = true;
  state.payload_id = "tote_alpha";
  state.state = "LOADED";
  state.last_action = "confirm_load";
  state.message = "loaded tote_alpha at receiving";
  state.weight_kg = 12.5;
  return state;
}

}  // namespace

TEST(PayloadWorkflowTest, ConfirmLoad_EmptyPayload_LoadsAndPlansEvent) {
  const auto decision =
      robot_tasks::PlanConfirmLoad(EmptyState(), "tote_alpha", "receiving", 12.5);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "LOADED");
  EXPECT_EQ(decision.last_action, "confirm_load");
  EXPECT_EQ(decision.event_state, "PAYLOAD_LOADED");
  EXPECT_EQ(decision.event_previous_state, "EMPTY");
}

TEST(PayloadWorkflowTest, ConfirmLoad_WhenAlreadyLoaded_RejectsWithoutMutation) {
  const auto decision =
      robot_tasks::PlanConfirmLoad(LoadedState(), "tote_beta", "receiving", 10.0);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "LOADED");
  EXPECT_EQ(decision.message, "payload already loaded: tote_alpha");
}

TEST(PayloadWorkflowTest, ConfirmLoad_Overweight_RejectsWithoutMutation) {
  const auto decision =
      robot_tasks::PlanConfirmLoad(EmptyState(), "tote_heavy", "receiving", 35.0);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_FALSE(decision.loaded);
  EXPECT_TRUE(decision.payload_id.empty());
  EXPECT_EQ(decision.state, "EMPTY");
  EXPECT_NE(decision.message.find("payload overweight"), std::string::npos);
}

TEST(PayloadWorkflowTest, ConfirmUnload_Mismatch_RejectsCurrentPayload) {
  const auto decision =
      robot_tasks::PlanConfirmUnload(LoadedState(), "wrong_tote", "packing");

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "LOADED");
  EXPECT_EQ(decision.message, "loaded payload mismatch: tote_alpha");
}

TEST(PayloadWorkflowTest, ConfirmUnload_LoadedPayload_EmptiesAndPlansEvent) {
  const auto decision =
      robot_tasks::PlanConfirmUnload(LoadedState(), "tote_alpha", "packing");

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_FALSE(decision.loaded);
  EXPECT_EQ(decision.state, "EMPTY");
  EXPECT_EQ(decision.last_action, "confirm_unload");
  EXPECT_EQ(decision.event_state, "PAYLOAD_UNLOADED");
  EXPECT_EQ(decision.event_previous_state, "LOADED");
}

TEST(PayloadWorkflowTest, SetPayloadLoaded_Overweight_RejectsWithoutMutation) {
  const auto decision =
      robot_tasks::PlanSetPayloadLoaded(LoadedState(), true, "tote_heavy", 35.0, "");

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "LOADED");
}

TEST(PayloadWorkflowTest, TopModuleAction_Unsupported_RejectsWithoutMutation) {
  auto state = LoadedState();
  state.top_module_type = "lift";
  const auto decision = robot_tasks::PlanTopModuleAction(state, "attach");

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "LOADED");
  EXPECT_EQ(decision.message, "unsupported top module action attach for lift");
}

TEST(PayloadWorkflowTest, TopModuleAction_Supported_UpdatesActionState) {
  auto state = LoadedState();
  state.top_module_type = "lift";
  const auto decision = robot_tasks::PlanTopModuleAction(state, "lift_up");

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_TRUE(decision.loaded);
  EXPECT_EQ(decision.payload_id, "tote_alpha");
  EXPECT_EQ(decision.state, "TOP_MODULE_ACTION");
  EXPECT_EQ(decision.last_action, "lift_up");
  EXPECT_EQ(decision.event_mission_id, "top_module_lift_up");
}
