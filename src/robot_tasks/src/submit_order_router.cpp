#include "robot_tasks/submit_order_router.hpp"

#include <algorithm>
#include <cctype>

#include "robot_tasks/order_payload.hpp"

namespace robot_tasks {
namespace {

bool IsOneOf(const std::string& value, std::initializer_list<const char*> aliases) {
  return std::any_of(aliases.begin(), aliases.end(), [&value](const char* alias) {
    return value == alias;
  });
}

}  // namespace

std::string NormalizeSubmitOrderType(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const char ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) != 0) {
      result.push_back(static_cast<char>(std::tolower(uch)));
    } else if (ch == '-' || ch == '_' || ch == '/') {
      result.push_back('_');
    }
  }
  return result;
}

std::optional<SubmitOrderRoute> BuildSubmitOrderRoute(
    const SubmitOrderInput& input, std::string* message) {
  std::string payload_message;
  const auto payload = ParseOrderPayload(input.payload_json, &payload_message);
  if (!payload.has_value()) {
    *message = "invalid payload_json: " + payload_message;
    return std::nullopt;
  }

  SubmitOrderRoute route;
  route.normalized_order_type = NormalizeSubmitOrderType(input.order_type);
  route.order_id = payload->GetString("order_id", input.order_id);
  route.priority = payload->GetInt("priority", input.priority);
  route.start_if_idle = payload->GetBool("start_if_idle", true);
  route.preempt_current = payload->GetBool("preempt_current", false);

  if (route.normalized_order_type.empty()) {
    *message = "order_type is empty";
    return std::nullopt;
  }

  if (IsOneOf(route.normalized_order_type, {"transport", "transport_order"})) {
    route.kind = SubmitOrderKind::kTransport;
    route.frame_id = payload->GetString("frame_id", "map");
    route.pickup_x = payload->GetDouble("pickup_x", 0.0);
    route.pickup_y = payload->GetDouble("pickup_y", 0.0);
    route.pickup_yaw = payload->GetDouble("pickup_yaw", 0.0);
    route.dropoff_x = payload->GetDouble("dropoff_x", 0.0);
    route.dropoff_y = payload->GetDouble("dropoff_y", 0.0);
    route.dropoff_yaw = payload->GetDouble("dropoff_yaw", 0.0);
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"station_transport", "station_transport_order"})) {
    route.kind = SubmitOrderKind::kStationTransport;
    route.pickup_station = payload->GetString("pickup_station");
    route.dropoff_station = payload->GetString("dropoff_station");
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"station_batch", "station_order_batch"})) {
    route.kind = SubmitOrderKind::kStationOrderBatch;
    route.batch_id = payload->GetString("batch_id", route.order_id);
    route.order_ids = payload->GetStringList("order_ids");
    route.pickup_station_ids =
        payload->GetStringList("pickup_station_ids", payload->GetStringList("pickup_stations"));
    route.dropoff_station_ids =
        payload->GetStringList("dropoff_station_ids", payload->GetStringList("dropoff_stations"));
    route.continue_on_error = payload->GetBool("continue_on_error", false);
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"station_sequence", "station_sequence_task"})) {
    route.kind = SubmitOrderKind::kStationSequence;
    route.station_ids =
        payload->GetStringList("station_ids", payload->GetStringList("station_list"));
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"fleet_station", "fleet_station_task"})) {
    route.kind = SubmitOrderKind::kFleetStation;
    route.task_id = payload->GetString("task_id", route.order_id);
    route.required_capability = payload->GetString("required_capability", "station_transport");
    route.pickup_station = payload->GetString("pickup_station");
    route.dropoff_station = payload->GetString("dropoff_station");
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"business", "business_order"})) {
    route.kind = SubmitOrderKind::kBusiness;
    route.business_order_id = payload->GetString("business_order_id", route.order_id);
    route.business_type = payload->GetString("business_type");
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"scenario", "scenario_task"})) {
    route.kind = SubmitOrderKind::kScenarioTask;
    route.scenario_id = payload->GetString("scenario_id");
    route.task_id = payload->GetString("task_id");
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"scenario_workflow", "workflow"})) {
    route.kind = SubmitOrderKind::kScenarioWorkflow;
    route.scenario_id = payload->GetString("scenario_id");
    route.workflow_id = payload->GetString("workflow_id");
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"vda5050", "vda_5050"})) {
    route.kind = SubmitOrderKind::kVda5050;
    route.serial_number = payload->GetString("serial_number");
    route.pickup_node_id =
        payload->GetString("pickup_node_id", payload->GetString("pickup_station"));
    route.dropoff_node_id =
        payload->GetString("dropoff_node_id", payload->GetString("dropoff_station"));
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"picking_wave", "picking"})) {
    route.kind = SubmitOrderKind::kPickingWave;
    route.wave_id = payload->GetString("wave_id", route.order_id);
    route.sku_id = payload->GetString("sku_id");
    route.tote_id = payload->GetString("tote_id");
    route.pick_station = payload->GetString("pick_station", payload->GetString("pickup_station"));
    route.dropoff_station = payload->GetString("dropoff_station");
    route.quantity = payload->GetInt("quantity", 1);
    return route;
  }

  if (IsOneOf(route.normalized_order_type, {"replenishment", "replenishment_order"})) {
    route.kind = SubmitOrderKind::kReplenishment;
    route.sku_id = payload->GetString("sku_id");
    route.tote_id = payload->GetString("tote_id");
    route.source_station =
        payload->GetString("source_station", payload->GetString("pickup_station"));
    route.line_station = payload->GetString("line_station", payload->GetString("dropoff_station"));
    route.quantity = payload->GetInt("quantity", 1);
    return route;
  }

  if (route.normalized_order_type == "milk_run") {
    route.kind = SubmitOrderKind::kMilkRun;
    route.run_id = payload->GetString("run_id", route.order_id);
    route.tote_id = payload->GetString("tote_id");
    route.station_ids =
        payload->GetStringList("station_ids", payload->GetStringList("station_list"));
    return route;
  }

  *message = "unsupported order_type: " + input.order_type +
             " (supported: transport, station_transport, station_order_batch, "
             "station_sequence, fleet_station, business, scenario_task, scenario_workflow, "
             "vda5050, picking_wave, replenishment, milk_run)";
  return std::nullopt;
}

Vda5050StationOrderDecision PlanVda5050StationOrder(
    const Vda5050StationOrderRequest& request) {
  Vda5050StationOrderDecision decision;
  decision.order_id = request.order_id;
  decision.serial_number = request.serial_number;
  decision.priority = request.priority;
  decision.start_if_idle = request.start_if_idle;
  decision.preempt_current = request.preempt_current;

  if (request.order_id.empty()) {
    decision.message = "vda5050 order_id is empty";
    return decision;
  }
  if (request.pickup_node_id.empty() || request.dropoff_node_id.empty()) {
    decision.message = "vda5050 pickup_node_id or dropoff_node_id is empty";
    return decision;
  }
  if (request.pickup_node_id == request.dropoff_node_id) {
    decision.message =
        "vda5050 pickup and dropoff node are identical: " + request.pickup_node_id;
    return decision;
  }

  decision.success = true;
  decision.station_order_id = "vda5050_" + request.order_id;
  decision.pickup_station = request.pickup_node_id;
  decision.dropoff_station = request.dropoff_node_id;
  decision.message = "accepted vda5050 order " + request.order_id;
  return decision;
}

}  // namespace robot_tasks
