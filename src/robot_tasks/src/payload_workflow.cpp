#include "robot_tasks/payload_workflow.hpp"

#include <string>

namespace robot_tasks {
namespace {

PayloadWorkflowDecision CurrentStateDecision(
    const PayloadWorkflowState& state, const std::string& message) {
  PayloadWorkflowDecision decision;
  decision.loaded = state.loaded;
  decision.payload_id = state.payload_id;
  decision.state = state.state;
  decision.last_action = state.last_action;
  decision.message = message;
  decision.weight_kg = state.weight_kg;
  return decision;
}

bool PayloadWeightAllowed(
    const PayloadWorkflowState& state, const double weight_kg, std::string* message) {
  if (weight_kg < 0.0) {
    if (message != nullptr) {
      *message = "payload weight must be non-negative";
    }
    return false;
  }
  if (weight_kg > state.capacity_kg) {
    if (message != nullptr) {
      *message = "payload overweight: " + std::to_string(weight_kg) + "kg > capacity " +
                 std::to_string(state.capacity_kg) + "kg";
    }
    return false;
  }
  return true;
}

}  // namespace

bool TopModuleActionSupported(
    const std::string& top_module_type, const std::string& action) {
  if (top_module_type == "manual_load") {
    return action == "wait_load" || action == "wait_unload" || action == "inspect";
  }
  if (top_module_type == "lift") {
    return action == "lift_up" || action == "lift_down" || action == "inspect";
  }
  if (top_module_type == "roller" || top_module_type == "conveyor") {
    return action == "load" || action == "unload" || action == "stop" ||
           action == "inspect";
  }
  if (top_module_type == "cart_tow") {
    return action == "attach" || action == "detach" || action == "inspect";
  }
  return action == "inspect";
}

PayloadWorkflowDecision PlanConfirmLoad(
    const PayloadWorkflowState& state, const std::string& payload_id,
    const std::string& station_id, const double weight_kg) {
  if (state.loaded) {
    return CurrentStateDecision(state, "payload already loaded: " + state.payload_id);
  }

  std::string message;
  if (!PayloadWeightAllowed(state, weight_kg, &message)) {
    return CurrentStateDecision(state, message);
  }

  PayloadWorkflowDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.loaded = true;
  decision.payload_id = payload_id;
  decision.weight_kg = weight_kg;
  decision.state = "LOADED";
  decision.last_action = "confirm_load";
  decision.message = "loaded " + payload_id + " at " + station_id;
  decision.event_mission_id = "payload_" + payload_id;
  decision.event_state = "PAYLOAD_LOADED";
  decision.event_previous_state = state.state;
  decision.event_message = decision.message;
  return decision;
}

PayloadWorkflowDecision PlanConfirmUnload(
    const PayloadWorkflowState& state, const std::string& requested_payload_id,
    const std::string& station_id) {
  if (!state.loaded) {
    PayloadWorkflowDecision decision = CurrentStateDecision(state, "payload already empty");
    decision.success = true;
    decision.payload_id = requested_payload_id;
    decision.state = "EMPTY";
    decision.loaded = false;
    decision.weight_kg = 0.0;
    return decision;
  }
  if (!requested_payload_id.empty() && requested_payload_id != state.payload_id) {
    return CurrentStateDecision(state, "loaded payload mismatch: " + state.payload_id);
  }

  PayloadWorkflowDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.loaded = false;
  decision.payload_id = "";
  decision.weight_kg = 0.0;
  decision.state = "EMPTY";
  decision.last_action = "confirm_unload";
  decision.message = "unloaded " + state.payload_id + " at " + station_id;
  decision.event_mission_id = "payload_" + state.payload_id;
  decision.event_state = "PAYLOAD_UNLOADED";
  decision.event_previous_state = state.state;
  decision.event_message = decision.message;
  return decision;
}

PayloadWorkflowDecision PlanSetPayloadLoaded(
    const PayloadWorkflowState& state, const bool loaded, const std::string& payload_id,
    const double weight_kg, const std::string& message) {
  std::string validation_message;
  if (loaded && !PayloadWeightAllowed(state, weight_kg, &validation_message)) {
    return CurrentStateDecision(state, validation_message);
  }

  PayloadWorkflowDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.loaded = loaded;
  decision.payload_id = loaded ? payload_id : "";
  decision.weight_kg = loaded ? weight_kg : 0.0;
  decision.state = loaded ? "LOADED" : "EMPTY";
  decision.last_action = "set_payload_loaded";
  decision.message = message.empty() ? "payload state set" : message;
  decision.event_mission_id = loaded ? "payload_" + payload_id : "payload_state";
  decision.event_state = decision.state;
  decision.event_previous_state = state.state;
  decision.event_message = decision.message;
  return decision;
}

PayloadWorkflowDecision PlanTopModuleAction(
    const PayloadWorkflowState& state, const std::string& action) {
  if (!TopModuleActionSupported(state.top_module_type, action)) {
    return CurrentStateDecision(
        state, "unsupported top module action " + action + " for " +
                   state.top_module_type);
  }

  PayloadWorkflowDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.loaded = state.loaded;
  decision.payload_id = state.payload_id;
  decision.weight_kg = state.weight_kg;
  decision.state = "TOP_MODULE_ACTION";
  decision.last_action = action;
  decision.message = "executed " + action + " on " + state.top_module_type;
  decision.event_mission_id = "top_module_" + action;
  decision.event_state = "TOP_MODULE_ACTION";
  decision.event_previous_state = state.state;
  decision.event_message = decision.message;
  return decision;
}

}  // namespace robot_tasks
