#pragma once

#include <string>

namespace robot_tasks {

struct PayloadWorkflowState {
  bool loaded = false;
  std::string payload_id;
  std::string state = "EMPTY";
  std::string last_action = "none";
  std::string message = "payload empty";
  double weight_kg = 0.0;
  double capacity_kg = 0.0;
  std::string top_module_type = "manual_load";
};

struct PayloadWorkflowDecision {
  bool success = false;
  bool mutate_state = false;
  bool loaded = false;
  std::string payload_id;
  std::string state = "EMPTY";
  std::string last_action = "none";
  std::string message;
  double weight_kg = 0.0;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

PayloadWorkflowDecision PlanConfirmLoad(
    const PayloadWorkflowState& state, const std::string& payload_id,
    const std::string& station_id, double weight_kg);
PayloadWorkflowDecision PlanConfirmUnload(
    const PayloadWorkflowState& state, const std::string& requested_payload_id,
    const std::string& station_id);
PayloadWorkflowDecision PlanSetPayloadLoaded(
    const PayloadWorkflowState& state, bool loaded, const std::string& payload_id,
    double weight_kg, const std::string& message);
PayloadWorkflowDecision PlanTopModuleAction(
    const PayloadWorkflowState& state, const std::string& action);

bool TopModuleActionSupported(
    const std::string& top_module_type, const std::string& action);

}  // namespace robot_tasks
