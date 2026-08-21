#include "robot_tasks/station_confirmation_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

StationConfirmationRuntimeState MakeConfirmation(const std::string& policy) {
  StationConfirmationRuntimeState confirmation;
  confirmation.confirmation_id = "confirm_a";
  confirmation.station_id = "packing";
  confirmation.action = "unload";
  confirmation.timeout_sec = 10;
  confirmation.created_at_sec = 5.0;
  confirmation.timeout_policy = policy;
  return confirmation;
}

StationConfirmationRequest MakeRequest() {
  StationConfirmationRequest request;
  request.confirmation_id = "confirm_a";
  request.station_id = "packing";
  request.action = "unload";
  request.mission_id = "mission_a";
  request.operator_hint = "check tote seal";
  request.timeout_sec = 30;
  request.timeout_policy = "retry";
  request.created_at_sec = 12.0;
  request.station_known = true;
  return request;
}

}  // namespace

TEST(StationConfirmationWorkflowTest, NormalizeTimeoutPolicy_UnknownFallsBackToManual) {
  EXPECT_EQ(NormalizeStationConfirmationTimeoutPolicy("retry"), "retry");
  EXPECT_EQ(NormalizeStationConfirmationTimeoutPolicy("skip"), "skip");
  EXPECT_EQ(NormalizeStationConfirmationTimeoutPolicy("return_to_dock"), "return_to_dock");
  EXPECT_EQ(NormalizeStationConfirmationTimeoutPolicy("manual"), "manual");
  EXPECT_EQ(NormalizeStationConfirmationTimeoutPolicy("surprise"), "manual");
}

TEST(StationConfirmationWorkflowTest, PlanRequest_EmptyStationRejects) {
  auto request = MakeRequest();
  request.station_id.clear();

  const auto decision = PlanStationConfirmationRequest(request, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.confirmation_id, "confirm_a");
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.message, "station_id is empty");
}

TEST(StationConfirmationWorkflowTest, PlanRequest_UnknownStationRejects) {
  auto request = MakeRequest();
  request.station_known = false;

  const auto decision = PlanStationConfirmationRequest(request, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.message, "unknown station: packing");
}

TEST(StationConfirmationWorkflowTest, PlanRequest_DuplicatePendingRejectsWithoutMutation) {
  auto request = MakeRequest();
  StationConfirmationStateMap confirmations;
  confirmations["confirm_a"] = MakeConfirmation("manual");

  const auto decision = PlanStationConfirmationRequest(request, confirmations);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.state, "PENDING");
  EXPECT_EQ(decision.message, "confirmation already pending: confirm_a");
}

TEST(StationConfirmationWorkflowTest, PlanRequest_ValidRequestCreatesPendingEvent) {
  const auto request = MakeRequest();

  const auto decision = PlanStationConfirmationRequest(request, {});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.confirmation_id, "confirm_a");
  EXPECT_EQ(decision.state, "PENDING");
  EXPECT_EQ(decision.message, "station confirmation pending: confirm_a");
  EXPECT_EQ(decision.confirmation.confirmation_id, "confirm_a");
  EXPECT_EQ(decision.confirmation.station_id, "packing");
  EXPECT_EQ(decision.confirmation.action, "unload");
  EXPECT_EQ(decision.confirmation.mission_id, "mission_a");
  EXPECT_EQ(decision.confirmation.operator_hint, "check tote seal");
  EXPECT_EQ(decision.confirmation.timeout_sec, 30);
  EXPECT_DOUBLE_EQ(decision.confirmation.created_at_sec, 12.0);
  EXPECT_EQ(decision.confirmation.timeout_policy, "retry");
  EXPECT_EQ(decision.event_mission_id, "station_confirmation_confirm_a");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_PENDING");
  EXPECT_EQ(decision.event_previous_state, "READY");
  EXPECT_EQ(decision.event_message, "pending unload at packing");
  EXPECT_TRUE(decision.event_recoverable);
}

TEST(StationConfirmationWorkflowTest, PlanRequest_NegativeTimeoutClampsAndUnknownPolicyDefaults) {
  auto request = MakeRequest();
  request.timeout_sec = -5;
  request.timeout_policy = "surprise";

  const auto decision = PlanStationConfirmationRequest(request, {});

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.confirmation.timeout_sec, 0);
  EXPECT_EQ(decision.confirmation.timeout_policy, "manual");
}

TEST(StationConfirmationWorkflowTest, PlanResolve_UnknownConfirmationRejects) {
  StationConfirmationResolveRequest request;
  request.confirmation_id = "missing";
  request.state = "CONFIRMED";
  request.operator_id = "alice";
  request.note = "confirmed";

  const auto decision = PlanStationConfirmationResolve(request, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.confirmation_id, "missing");
  EXPECT_EQ(decision.state, "UNKNOWN");
  EXPECT_EQ(decision.message, "unknown confirmation: missing");
}

TEST(StationConfirmationWorkflowTest, PlanResolve_AlreadyResolvedRejects) {
  StationConfirmationStateMap confirmations;
  auto confirmation = MakeConfirmation("manual");
  confirmation.state = "CONFIRMED";
  confirmations["confirm_a"] = confirmation;
  StationConfirmationResolveRequest request;
  request.confirmation_id = "confirm_a";
  request.state = "REJECTED";
  request.operator_id = "alice";
  request.note = "rejected";

  const auto decision = PlanStationConfirmationResolve(request, confirmations);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.state, "CONFIRMED");
  EXPECT_EQ(decision.message, "confirmation already resolved: confirm_a");
}

TEST(StationConfirmationWorkflowTest, PlanResolve_ConfirmCreatesRecoverableEvent) {
  StationConfirmationStateMap confirmations;
  confirmations["confirm_a"] = MakeConfirmation("manual");
  StationConfirmationResolveRequest request;
  request.confirmation_id = "confirm_a";
  request.state = "CONFIRMED";
  request.operator_id = "alice";
  request.note = "station action confirmed";

  const auto decision = PlanStationConfirmationResolve(request, confirmations);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.confirmation_id, "confirm_a");
  EXPECT_EQ(decision.state, "CONFIRMED");
  EXPECT_EQ(decision.message, "station action confirmed");
  EXPECT_EQ(decision.event_mission_id, "station_confirmation_confirm_a");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_CONFIRMED");
  EXPECT_EQ(decision.event_previous_state, "PENDING");
  EXPECT_EQ(decision.event_message, "station action confirmed by alice");
  EXPECT_TRUE(decision.event_recoverable);
}

TEST(StationConfirmationWorkflowTest, PlanResolve_RejectCreatesNonRecoverableEvent) {
  StationConfirmationStateMap confirmations;
  confirmations["confirm_a"] = MakeConfirmation("manual");
  StationConfirmationResolveRequest request;
  request.confirmation_id = "confirm_a";
  request.state = "REJECTED";
  request.operator_id = "alice";
  request.note = "station action rejected";

  const auto decision = PlanStationConfirmationResolve(request, confirmations);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_REJECTED");
  EXPECT_EQ(decision.event_message, "station action rejected by alice");
  EXPECT_FALSE(decision.event_recoverable);
}

TEST(StationConfirmationWorkflowTest, BuildPendingProjection_FiltersPendingByStation) {
  StationConfirmationStateMap confirmations;
  auto pending = MakeConfirmation("manual");
  pending.mission_id = "mission_a";
  pending.operator_hint = "check tote";
  confirmations["confirm_a"] = pending;
  auto resolved = MakeConfirmation("manual");
  resolved.state = "CONFIRMED";
  confirmations["confirm_resolved"] = resolved;
  auto other_station = MakeConfirmation("manual");
  other_station.station_id = "storage";
  confirmations["confirm_storage"] = other_station;

  const auto projection = BuildPendingStationConfirmationsProjection(confirmations, "packing");

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.confirmation_ids, (std::vector<std::string>{"confirm_a"}));
  EXPECT_EQ(projection.station_ids, (std::vector<std::string>{"packing"}));
  EXPECT_EQ(projection.actions, (std::vector<std::string>{"unload"}));
  EXPECT_EQ(projection.mission_ids, (std::vector<std::string>{"mission_a"}));
  EXPECT_EQ(projection.operator_hints, (std::vector<std::string>{"check tote"}));
  EXPECT_EQ(projection.states, (std::vector<std::string>{"PENDING"}));
  EXPECT_EQ(projection.message, "loaded 1 pending confirmation(s)");
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_NotPending_DoesNothing) {
  auto confirmation = MakeConfirmation("manual");
  confirmation.state = "CONFIRMED";

  const auto decision = PlanStationConfirmationTimeout(confirmation, 20.0);

  EXPECT_FALSE(decision.timed_out);
  EXPECT_FALSE(decision.update_state);
  EXPECT_FALSE(decision.set_runner_state);
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_NotExpired_DoesNothing) {
  const auto confirmation = MakeConfirmation("manual");

  const auto decision = PlanStationConfirmationTimeout(confirmation, 14.0);

  EXPECT_FALSE(decision.timed_out);
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_Retry_RefreshesCreatedAtAndKeepsPending) {
  const auto confirmation = MakeConfirmation("retry");

  const auto decision = PlanStationConfirmationTimeout(confirmation, 20.0);

  EXPECT_TRUE(decision.timed_out);
  EXPECT_TRUE(decision.update_created_at);
  EXPECT_DOUBLE_EQ(decision.new_created_at_sec, 20.0);
  EXPECT_FALSE(decision.update_state);
  EXPECT_EQ(decision.event_mission_id, "station_confirmation_confirm_a");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_TIMEOUT_RETRY");
  EXPECT_TRUE(decision.event_recoverable);
  EXPECT_FALSE(decision.request_dock_return);
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_Skip_MarksSkipped) {
  const auto confirmation = MakeConfirmation("skip");

  const auto decision = PlanStationConfirmationTimeout(confirmation, 20.0);

  EXPECT_TRUE(decision.timed_out);
  EXPECT_TRUE(decision.update_state);
  EXPECT_EQ(decision.state, "SKIPPED");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_SKIPPED");
  EXPECT_TRUE(decision.event_recoverable);
  EXPECT_FALSE(decision.set_runner_state);
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_ReturnToDock_MarksTimedOutAndRequestsDock) {
  const auto confirmation = MakeConfirmation("return_to_dock");

  const auto decision = PlanStationConfirmationTimeout(confirmation, 20.0);

  EXPECT_TRUE(decision.timed_out);
  EXPECT_TRUE(decision.update_state);
  EXPECT_EQ(decision.state, "TIMED_OUT");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_TIMED_OUT");
  EXPECT_TRUE(decision.event_recoverable);
  EXPECT_TRUE(decision.request_dock_return);
  EXPECT_FALSE(decision.set_runner_state);
}

TEST(StationConfirmationWorkflowTest, PlanTimeout_Manual_MarksTimedOutAndNeedsOperator) {
  const auto confirmation = MakeConfirmation("manual");

  const auto decision = PlanStationConfirmationTimeout(confirmation, 20.0);

  EXPECT_TRUE(decision.timed_out);
  EXPECT_TRUE(decision.update_state);
  EXPECT_EQ(decision.state, "TIMED_OUT");
  EXPECT_EQ(decision.event_state, "CONFIRMATION_TIMED_OUT");
  EXPECT_FALSE(decision.event_recoverable);
  EXPECT_TRUE(decision.set_runner_state);
  EXPECT_EQ(decision.runner_state, "NEEDS_OPERATOR");
  EXPECT_TRUE(decision.runner_recoverable);
}

}  // namespace robot_tasks
