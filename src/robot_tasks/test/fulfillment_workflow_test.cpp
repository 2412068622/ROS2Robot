#include "robot_tasks/fulfillment_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(FulfillmentWorkflowTest, PlanFulfillmentSubmit_NewOrderAccepted) {
  FulfillmentSubmitRequest request;
  request.fulfillment_id = "milk_run_north_loop";
  request.order_type = "milk_run";
  request.station_count = 3;
  request.require_two_stations = true;

  const auto decision = PlanFulfillmentSubmit(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.fulfillment_id, "milk_run_north_loop");
  EXPECT_EQ(decision.state, "ACCEPTED");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentSubmit_DuplicateRejectedBeforeQueueing) {
  FulfillmentSubmitRequest request;
  request.fulfillment_id = "milk_run_north_loop";
  request.order_type = "milk_run";
  request.station_count = 3;
  request.duplicate = true;
  request.require_two_stations = true;

  const auto decision = PlanFulfillmentSubmit(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "DUPLICATE");
  EXPECT_EQ(decision.message, "fulfillment already exists: milk_run_north_loop");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentSubmit_MilkRunRequiresTwoStations) {
  FulfillmentSubmitRequest request;
  request.fulfillment_id = "milk_run_short";
  request.order_type = "milk_run";
  request.station_count = 1;
  request.require_two_stations = true;

  const auto decision = PlanFulfillmentSubmit(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.message, "fulfillment station sequence requires at least two stations");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentSubmit_MissingIdRejected) {
  FulfillmentSubmitRequest request;
  request.fulfillment_id = "picking_wave_";
  request.order_type = "picking_wave";
  request.station_count = 2;

  const auto decision = PlanFulfillmentSubmit(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.message, "fulfillment id is required");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentPick_AwaitingPick_ComputesShortPick) {
  FulfillmentPickRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "AWAITING_PICK";
  request.requested_quantity = 10;
  request.picked_quantity = 8;

  const auto decision = PlanFulfillmentPick(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.state, "SHORT_PICK");
  EXPECT_EQ(decision.picked_quantity, 8);
  EXPECT_EQ(decision.short_quantity, 2);
  EXPECT_TRUE(decision.resolve_pick_confirmation);
  EXPECT_EQ(decision.event_previous_state, "AWAITING_PICK");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentPick_DropoffReadyState_RejectsReplay) {
  FulfillmentPickRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "PICK_CONFIRMED";
  request.requested_quantity = 10;
  request.picked_quantity = 10;

  const auto decision = PlanFulfillmentPick(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "PICK_CONFIRMED");
  EXPECT_EQ(
      decision.message,
      "fulfillment pick is not pending: picking_wave_alpha state=PICK_CONFIRMED");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentPick_MilkRunRejected) {
  FulfillmentPickRequest request;
  request.fulfillment_id = "milk_run_alpha";
  request.order_type = "milk_run";
  request.current_state = "ROUTE_QUEUED";

  const auto decision = PlanFulfillmentPick(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(
      decision.message,
      "fulfillment does not support pick confirmation: milk_run_alpha");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentDropoff_PickConfirmed_Fulfilled) {
  FulfillmentDropoffRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "PICK_CONFIRMED";
  request.short_quantity = 0;
  request.confirmation_count = 2;

  const auto decision = PlanFulfillmentDropoff(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.state, "FULFILLED");
  EXPECT_TRUE(decision.resolve_dropoff_confirmation);
  EXPECT_FALSE(decision.resolve_all_confirmations);
  EXPECT_EQ(decision.event_previous_state, "PICK_CONFIRMED");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentDropoff_ShortPick_FulfilledWithShortPick) {
  FulfillmentDropoffRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "SHORT_PICK";
  request.short_quantity = 2;
  request.confirmation_count = 1;

  const auto decision = PlanFulfillmentDropoff(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.state, "FULFILLED_WITH_SHORT_PICK");
  EXPECT_FALSE(decision.resolve_dropoff_confirmation);
  EXPECT_TRUE(decision.resolve_all_confirmations);
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentDropoff_AwaitingPickRejected) {
  FulfillmentDropoffRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "AWAITING_PICK";

  const auto decision = PlanFulfillmentDropoff(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "AWAITING_PICK");
  EXPECT_EQ(
      decision.message,
      "fulfillment dropoff is not ready: picking_wave_alpha state=AWAITING_PICK");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentShortPick_AwaitingPick_ComputesPickedQuantity) {
  FulfillmentShortPickRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "AWAITING_PICK";
  request.requested_quantity = 10;
  request.short_quantity = 3;

  const auto decision = PlanFulfillmentShortPick(request);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.state, "SHORT_PICK");
  EXPECT_EQ(decision.short_quantity, 3);
  EXPECT_EQ(decision.picked_quantity, 7);
  EXPECT_EQ(decision.event_previous_state, "AWAITING_PICK");
}

TEST(FulfillmentWorkflowTest, PlanFulfillmentShortPick_AfterPickRejected) {
  FulfillmentShortPickRequest request;
  request.fulfillment_id = "picking_wave_alpha";
  request.order_type = "picking_wave";
  request.current_state = "PICK_CONFIRMED";
  request.short_quantity = 1;

  const auto decision = PlanFulfillmentShortPick(request);

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.state, "PICK_CONFIRMED");
  EXPECT_EQ(
      decision.message,
      "fulfillment short pick is not pending: picking_wave_alpha state=PICK_CONFIRMED");
}

TEST(FulfillmentWorkflowTest, BuildFulfillmentStatusProjection_CopiesRuntimeState) {
  FulfillmentOrderRuntimeState order;
  order.fulfillment_id = "picking_wave_alpha";
  order.order_type = "picking_wave";
  order.sku_id = "sku_42";
  order.tote_id = "tote_7";
  order.state = "SHORT_PICK";
  order.pick_station = "pick_a";
  order.dropoff_station = "drop_b";
  order.requested_quantity = 10;
  order.picked_quantity = 8;
  order.short_quantity = 2;
  order.mission_ids = {"mission_pick", "mission_drop"};
  order.confirmation_ids = {"confirm_pick", "confirm_drop"};

  const auto projection = BuildFulfillmentStatusProjection(order);

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.fulfillment_id, "picking_wave_alpha");
  EXPECT_EQ(projection.order_type, "picking_wave");
  EXPECT_EQ(projection.sku_id, "sku_42");
  EXPECT_EQ(projection.tote_id, "tote_7");
  EXPECT_EQ(projection.state, "SHORT_PICK");
  EXPECT_EQ(projection.pick_station, "pick_a");
  EXPECT_EQ(projection.dropoff_station, "drop_b");
  EXPECT_EQ(projection.requested_quantity, 10);
  EXPECT_EQ(projection.picked_quantity, 8);
  EXPECT_EQ(projection.short_quantity, 2);
  EXPECT_EQ(projection.mission_ids, (std::vector<std::string>{"mission_pick", "mission_drop"}));
  EXPECT_EQ(
      projection.confirmation_ids,
      (std::vector<std::string>{"confirm_pick", "confirm_drop"}));
  EXPECT_EQ(projection.message, "loaded fulfillment status: picking_wave_alpha");
}

}  // namespace robot_tasks
