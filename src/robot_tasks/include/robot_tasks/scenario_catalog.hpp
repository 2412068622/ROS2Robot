#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

struct ScenarioTask {
  std::string scenario_id;
  std::string task_id;
  std::string label;
  std::string task_type = "station_transport";
  std::string pickup_station_id;
  std::string dropoff_station_id;
  std::vector<std::string> station_sequence_ids;
  std::string required_capability = "station_transport";
  std::string resource_id;
  std::string resource_type;
  std::string action;
  bool hold_after_action = false;
  int default_priority = 30;
};

struct ScenarioWorkflow {
  std::string scenario_id;
  std::string workflow_id;
  std::string label;
  std::vector<std::string> task_ids;
  int default_priority = 30;
};

struct ScenarioCatalog {
  std::vector<ScenarioTask> tasks;
  std::vector<ScenarioWorkflow> workflows;
};

struct ScenarioWorkflowSubmitRequest {
  std::string scenario_id;
  std::string workflow_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

struct ScenarioWorkflowStepPlan {
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

struct ScenarioWorkflowSubmitDecision {
  bool success{false};
  std::string scenario_id;
  std::string workflow_id;
  std::vector<ScenarioWorkflowStepPlan> steps;
  std::string message;
};

enum class ScenarioTaskDispatchKind {
  kFleetStation,
  kFacilityAction,
  kStationSequence,
};

struct ScenarioTaskSubmitRequest {
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

struct ScenarioTaskSubmitDecision {
  bool success{false};
  ScenarioTaskDispatchKind dispatch_kind{ScenarioTaskDispatchKind::kFleetStation};
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string resource_id;
  std::string resource_type;
  std::string action;
  bool hold_after_action{false};
  std::vector<std::string> station_ids;
  std::string required_capability{"station_transport"};
  std::string pickup_station;
  std::string dropoff_station;
  std::string message;
};

std::optional<ScenarioCatalog> LoadScenarioCatalog(const std::filesystem::path& path);
const ScenarioTask* FindScenarioTask(
    const ScenarioCatalog& catalog, const std::string& scenario_id, const std::string& task_id);
const ScenarioWorkflow* FindScenarioWorkflow(
    const ScenarioCatalog& catalog, const std::string& scenario_id,
    const std::string& workflow_id);
bool ValidateScenarioCatalog(const ScenarioCatalog& catalog, std::string* message);
ScenarioWorkflowSubmitDecision PlanScenarioWorkflowSubmit(
    const ScenarioWorkflowSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message);
ScenarioTaskSubmitDecision PlanScenarioTaskSubmit(
    const ScenarioTaskSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message);

}  // namespace robot_tasks
