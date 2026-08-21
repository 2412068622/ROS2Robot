#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_tasks {

struct FulfillmentOrderRuntimeState {
  std::string fulfillment_id;
  std::string order_type;
  std::string sku_id;
  std::string tote_id;
  std::string state = "CREATED";
  std::string pick_station;
  std::string dropoff_station;
  int requested_quantity = 0;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::vector<std::string> mission_ids;
  std::vector<std::string> confirmation_ids;
};

struct FulfillmentSubmitRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::size_t station_count = 0;
  bool duplicate = false;
  bool require_two_stations = false;
};

struct FulfillmentSubmitDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
};

FulfillmentSubmitDecision PlanFulfillmentSubmit(
    const FulfillmentSubmitRequest& request);

struct FulfillmentPickRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int requested_quantity = 0;
  int picked_quantity = 0;
};

struct FulfillmentPickDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  int picked_quantity = 0;
  int short_quantity = 0;
  bool resolve_pick_confirmation = false;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

struct FulfillmentDropoffRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int short_quantity = 0;
  std::size_t confirmation_count = 0;
};

struct FulfillmentDropoffDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  bool resolve_dropoff_confirmation = false;
  bool resolve_all_confirmations = false;
  std::string event_state;
  std::string event_previous_state;
  bool event_recoverable = true;
};

struct FulfillmentShortPickRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int requested_quantity = 0;
  int short_quantity = 0;
};

struct FulfillmentShortPickDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::string event_state;
  std::string event_previous_state;
  bool event_recoverable = false;
};

struct FulfillmentStatusProjection {
  bool success = true;
  std::string fulfillment_id;
  std::string order_type;
  std::string sku_id;
  std::string tote_id;
  std::string state;
  std::string pick_station;
  std::string dropoff_station;
  int requested_quantity = 0;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::vector<std::string> mission_ids;
  std::vector<std::string> confirmation_ids;
  std::string message;
};

FulfillmentPickDecision PlanFulfillmentPick(
    const FulfillmentPickRequest& request);
FulfillmentDropoffDecision PlanFulfillmentDropoff(
    const FulfillmentDropoffRequest& request);
FulfillmentShortPickDecision PlanFulfillmentShortPick(
    const FulfillmentShortPickRequest& request);
FulfillmentStatusProjection BuildFulfillmentStatusProjection(
    const FulfillmentOrderRuntimeState& order);

}  // namespace robot_tasks
