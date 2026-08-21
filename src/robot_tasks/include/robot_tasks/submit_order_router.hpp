#pragma once

#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

enum class SubmitOrderKind {
  kTransport,
  kStationTransport,
  kStationOrderBatch,
  kStationSequence,
  kFleetStation,
  kBusiness,
  kScenarioTask,
  kScenarioWorkflow,
  kVda5050,
  kPickingWave,
  kReplenishment,
  kMilkRun,
};

struct SubmitOrderInput {
  std::string order_id;
  std::string order_type;
  int priority{0};
  std::string payload_json;
  std::vector<std::string> tags;
};

struct SubmitOrderRoute {
  SubmitOrderKind kind{SubmitOrderKind::kTransport};
  std::string normalized_order_type;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  bool continue_on_error{false};
  int quantity{1};
  std::string frame_id{"map"};
  double pickup_x{0.0};
  double pickup_y{0.0};
  double pickup_yaw{0.0};
  double dropoff_x{0.0};
  double dropoff_y{0.0};
  double dropoff_yaw{0.0};
  std::string pickup_station;
  std::string dropoff_station;
  std::string batch_id;
  std::vector<std::string> order_ids;
  std::vector<std::string> pickup_station_ids;
  std::vector<std::string> dropoff_station_ids;
  std::vector<std::string> station_ids;
  std::string task_id;
  std::string required_capability{"station_transport"};
  std::string business_order_id;
  std::string business_type;
  std::string scenario_id;
  std::string workflow_id;
  std::string serial_number;
  std::string pickup_node_id;
  std::string dropoff_node_id;
  std::string wave_id;
  std::string sku_id;
  std::string tote_id;
  std::string pick_station;
  std::string source_station;
  std::string line_station;
  std::string run_id;
};

struct Vda5050StationOrderRequest {
  std::string order_id;
  std::string serial_number;
  std::string pickup_node_id;
  std::string dropoff_node_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

struct Vda5050StationOrderDecision {
  bool success{false};
  std::string order_id;
  std::string serial_number;
  std::string station_order_id;
  std::string pickup_station;
  std::string dropoff_station;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string message;
};

std::string NormalizeSubmitOrderType(const std::string& value);
std::optional<SubmitOrderRoute> BuildSubmitOrderRoute(
    const SubmitOrderInput& input, std::string* message);
Vda5050StationOrderDecision PlanVda5050StationOrder(
    const Vda5050StationOrderRequest& request);

}  // namespace robot_tasks
