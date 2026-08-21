#include "robot_tasks/fulfillment_workflow.hpp"

namespace robot_tasks {
namespace {

bool IsPickDropoffFulfillment(const std::string& order_type) {
  return order_type == "picking_wave" || order_type == "replenishment";
}

bool IsPickPendingState(const std::string& state) {
  return state == "AWAITING_PICK";
}

bool IsDropoffReadyState(const std::string& state) {
  return state == "PICK_CONFIRMED" || state == "SHORT_PICK";
}

}  // namespace

FulfillmentSubmitDecision PlanFulfillmentSubmit(
    const FulfillmentSubmitRequest& request) {
  FulfillmentSubmitDecision decision;
  decision.fulfillment_id = request.fulfillment_id;

  if (request.fulfillment_id == request.order_type + "_") {
    decision.message = "fulfillment id is required";
    return decision;
  }
  if (request.duplicate) {
    decision.state = "DUPLICATE";
    decision.message = "fulfillment already exists: " + request.fulfillment_id;
    return decision;
  }
  if (request.require_two_stations && request.station_count < 2U) {
    decision.message = "fulfillment station sequence requires at least two stations";
    return decision;
  }

  decision.success = true;
  decision.state = "ACCEPTED";
  decision.message = "fulfillment submit accepted: " + request.fulfillment_id;
  return decision;
}

FulfillmentPickDecision PlanFulfillmentPick(
    const FulfillmentPickRequest& request) {
  FulfillmentPickDecision decision;
  decision.fulfillment_id = request.fulfillment_id;

  if (!IsPickDropoffFulfillment(request.order_type)) {
    decision.message =
        "fulfillment does not support pick confirmation: " + request.fulfillment_id;
    return decision;
  }
  if (!IsPickPendingState(request.current_state)) {
    decision.state = request.current_state;
    decision.message =
        "fulfillment pick is not pending: " + request.fulfillment_id +
        " state=" + request.current_state;
    return decision;
  }

  decision.success = true;
  decision.picked_quantity = request.picked_quantity < 0 ? 0 : request.picked_quantity;
  decision.short_quantity =
      request.requested_quantity - decision.picked_quantity > 0
          ? request.requested_quantity - decision.picked_quantity
          : 0;
  decision.state = decision.short_quantity > 0 ? "SHORT_PICK" : "PICK_CONFIRMED";
  decision.message = "pick confirmed for " + request.fulfillment_id;
  decision.resolve_pick_confirmation = true;
  decision.event_state = "FULFILLMENT_PICK_CONFIRMED";
  decision.event_previous_state = request.current_state;
  decision.event_message =
      "picked " + std::to_string(decision.picked_quantity) + " of " +
      std::to_string(request.requested_quantity);
  return decision;
}

FulfillmentDropoffDecision PlanFulfillmentDropoff(
    const FulfillmentDropoffRequest& request) {
  FulfillmentDropoffDecision decision;
  decision.fulfillment_id = request.fulfillment_id;

  if (!IsPickDropoffFulfillment(request.order_type)) {
    decision.message =
        "fulfillment does not support dropoff confirmation: " + request.fulfillment_id;
    return decision;
  }
  if (!IsDropoffReadyState(request.current_state)) {
    decision.state = request.current_state;
    decision.message =
        "fulfillment dropoff is not ready: " + request.fulfillment_id +
        " state=" + request.current_state;
    return decision;
  }

  decision.success = true;
  decision.state =
      request.short_quantity > 0 ? "FULFILLED_WITH_SHORT_PICK" : "FULFILLED";
  decision.message = "dropoff confirmed for " + request.fulfillment_id;
  decision.resolve_dropoff_confirmation = request.confirmation_count >= 2U;
  decision.resolve_all_confirmations = request.confirmation_count > 0U &&
                                      request.confirmation_count < 2U;
  decision.event_state = "FULFILLMENT_DROPOFF_CONFIRMED";
  decision.event_previous_state = request.current_state;
  return decision;
}

FulfillmentShortPickDecision PlanFulfillmentShortPick(
    const FulfillmentShortPickRequest& request) {
  FulfillmentShortPickDecision decision;
  decision.fulfillment_id = request.fulfillment_id;

  if (!IsPickDropoffFulfillment(request.order_type)) {
    decision.message =
        "fulfillment does not support short pick: " + request.fulfillment_id;
    return decision;
  }
  if (!IsPickPendingState(request.current_state)) {
    decision.state = request.current_state;
    decision.message =
        "fulfillment short pick is not pending: " + request.fulfillment_id +
        " state=" + request.current_state;
    return decision;
  }

  decision.success = true;
  decision.short_quantity = request.short_quantity < 0 ? 0 : request.short_quantity;
  decision.picked_quantity =
      request.requested_quantity - decision.short_quantity > 0
          ? request.requested_quantity - decision.short_quantity
          : 0;
  decision.state = "SHORT_PICK";
  decision.message = "short pick reported for " + request.fulfillment_id;
  decision.event_state = "FULFILLMENT_SHORT_PICK";
  decision.event_previous_state = request.current_state;
  return decision;
}

FulfillmentStatusProjection BuildFulfillmentStatusProjection(
    const FulfillmentOrderRuntimeState& order) {
  FulfillmentStatusProjection projection;
  projection.fulfillment_id = order.fulfillment_id;
  projection.order_type = order.order_type;
  projection.sku_id = order.sku_id;
  projection.tote_id = order.tote_id;
  projection.state = order.state;
  projection.pick_station = order.pick_station;
  projection.dropoff_station = order.dropoff_station;
  projection.requested_quantity = order.requested_quantity;
  projection.picked_quantity = order.picked_quantity;
  projection.short_quantity = order.short_quantity;
  projection.mission_ids = order.mission_ids;
  projection.confirmation_ids = order.confirmation_ids;
  projection.message = "loaded fulfillment status: " + order.fulfillment_id;
  return projection;
}

}  // namespace robot_tasks
