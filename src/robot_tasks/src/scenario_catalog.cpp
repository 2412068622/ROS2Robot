#include "robot_tasks/scenario_catalog.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>

namespace robot_tasks {
namespace {

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<std::string> ValueAfterColon(const std::string& line, const std::string& key) {
  const std::string prefix = key + ":";
  if (line.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return Trim(line.substr(prefix.size()));
}

std::optional<int> ParseInt(const std::string& value) {
  std::istringstream parser(value);
  int result = 0;
  parser >> result;
  if (parser.fail()) {
    return std::nullopt;
  }
  return result;
}

bool ParseBool(const std::string& value) {
  const auto normalized = Trim(value);
  return normalized == "true" || normalized == "True" || normalized == "1" ||
         normalized == "yes";
}

std::vector<std::string> ParseCsv(const std::string& value) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

}  // namespace

std::optional<ScenarioCatalog> LoadScenarioCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  ScenarioCatalog catalog;
  ScenarioTask current;
  ScenarioWorkflow current_workflow;
  bool in_task = false;
  bool in_workflow = false;
  bool has_scenario = false;
  bool has_task = false;
  bool has_pickup = false;
  bool has_dropoff = false;
  bool has_workflow = false;
  bool has_workflow_tasks = false;
  std::string section;

  auto flush_task = [&]() {
    if (!in_task) {
      return;
    }
    const auto task_type = current.task_type.empty() ? "station_transport" : current.task_type;
    const bool has_required_fields =
        task_type == "facility_action"
            ? (!current.action.empty() &&
               (!current.resource_id.empty() || !current.resource_type.empty()))
            : (task_type == "station_sequence" ? current.station_sequence_ids.size() >= 2U
                                                : (has_pickup && has_dropoff));
    if (in_task && has_scenario && has_task && has_required_fields) {
      catalog.tasks.push_back(current);
    }
    current = ScenarioTask{};
    in_task = false;
    has_scenario = false;
    has_task = false;
    has_pickup = false;
    has_dropoff = false;
  };

  auto flush_workflow = [&]() {
    if (!in_workflow) {
      return;
    }
    if (in_workflow && has_scenario && has_workflow && has_workflow_tasks) {
      catalog.workflows.push_back(current_workflow);
    }
    current_workflow = ScenarioWorkflow{};
    in_workflow = false;
    has_scenario = false;
    has_workflow = false;
    has_workflow_tasks = false;
  };

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "tasks:" || line == "scenario_tasks:") {
      flush_workflow();
      flush_task();
      section = line.substr(0, line.size() - 1);
      continue;
    }
    if (line == "workflows:" || line == "scenario_workflows:") {
      flush_task();
      flush_workflow();
      section = line.substr(0, line.size() - 1);
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      if (section != "tasks" && section != "scenario_tasks" &&
          section != "workflows" && section != "scenario_workflows") {
        continue;
      }
      if (section == "tasks" || section == "scenario_tasks") {
        flush_workflow();
        flush_task();
        in_task = true;
      } else {
        flush_workflow();
        flush_task();
        in_workflow = true;
      }
      line = Trim(line.substr(2));
    }
    if (!in_task && !in_workflow) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "scenario_id")) {
      if (in_task) {
        current.scenario_id = *value;
        has_scenario = !current.scenario_id.empty();
      } else {
        current_workflow.scenario_id = *value;
        has_scenario = !current_workflow.scenario_id.empty();
      }
      continue;
    }
    if (in_workflow) {
      if (auto value = ValueAfterColon(line, "workflow_id")) {
        current_workflow.workflow_id = *value;
        has_workflow = !current_workflow.workflow_id.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "label")) {
        current_workflow.label = *value;
        continue;
      }
      if (auto value = ValueAfterColon(line, "task_ids")) {
        current_workflow.task_ids = ParseCsv(*value);
        has_workflow_tasks = !current_workflow.task_ids.empty();
        continue;
      }
      if (auto value = ValueAfterColon(line, "default_priority")) {
        if (auto parsed = ParseInt(*value)) {
          current_workflow.default_priority = *parsed;
        }
        continue;
      }
      continue;
    }
    if (auto value = ValueAfterColon(line, "task_id")) {
      current.task_id = *value;
      has_task = !current.task_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "label")) {
      current.label = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "task_type")) {
      current.task_type = value->empty() ? "station_transport" : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "pickup_station_id")) {
      current.pickup_station_id = *value;
      has_pickup = !current.pickup_station_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "dropoff_station_id")) {
      current.dropoff_station_id = *value;
      has_dropoff = !current.dropoff_station_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "station_sequence_ids")) {
      current.station_sequence_ids = ParseCsv(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "required_capability")) {
      current.required_capability = value->empty() ? "station_transport" : *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "resource_id")) {
      current.resource_id = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "resource_type")) {
      current.resource_type = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "action")) {
      current.action = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "hold_after_action")) {
      current.hold_after_action = ParseBool(*value);
      continue;
    }
    if (auto value = ValueAfterColon(line, "default_priority")) {
      if (auto parsed = ParseInt(*value)) {
        current.default_priority = *parsed;
      }
      continue;
    }
  }
  flush_workflow();
  flush_task();

  std::string message;
  if (!ValidateScenarioCatalog(catalog, &message)) {
    return std::nullopt;
  }
  return catalog;
}

const ScenarioTask* FindScenarioTask(
    const ScenarioCatalog& catalog, const std::string& scenario_id, const std::string& task_id) {
  for (const auto& task : catalog.tasks) {
    if (task.scenario_id == scenario_id && task.task_id == task_id) {
      return &task;
    }
  }
  return nullptr;
}

const ScenarioWorkflow* FindScenarioWorkflow(
    const ScenarioCatalog& catalog, const std::string& scenario_id,
    const std::string& workflow_id) {
  for (const auto& workflow : catalog.workflows) {
    if (workflow.scenario_id == scenario_id && workflow.workflow_id == workflow_id) {
      return &workflow;
    }
  }
  return nullptr;
}

bool ValidateScenarioCatalog(const ScenarioCatalog& catalog, std::string* message) {
  std::unordered_set<std::string> ids;
  for (const auto& task : catalog.tasks) {
    if (task.scenario_id.empty() || task.task_id.empty()) {
      if (message != nullptr) {
        *message = "scenario_id or task_id is empty";
      }
      return false;
    }
    const auto task_type = task.task_type.empty() ? "station_transport" : task.task_type;
    if (task_type == "station_transport" &&
        (task.pickup_station_id.empty() || task.dropoff_station_id.empty())) {
      if (message != nullptr) {
        *message = "scenario task station is empty: " + task.scenario_id + "/" + task.task_id;
      }
      return false;
    }
    if (task_type == "facility_action" &&
        ((task.resource_id.empty() && task.resource_type.empty()) || task.action.empty())) {
      if (message != nullptr) {
        *message = "scenario facility action is incomplete: " + task.scenario_id + "/" +
                   task.task_id;
      }
      return false;
    }
    if (task_type == "station_sequence" && task.station_sequence_ids.size() < 2U) {
      if (message != nullptr) {
        *message = "scenario station sequence is incomplete: " + task.scenario_id + "/" +
                   task.task_id;
      }
      return false;
    }
    if (task_type != "station_transport" && task_type != "facility_action" &&
        task_type != "station_sequence") {
      if (message != nullptr) {
        *message = "unsupported scenario task type: " + task.scenario_id + "/" +
                   task.task_id + "/" + task_type;
      }
      return false;
    }
    const auto key = task.scenario_id + "/" + task.task_id;
    if (!ids.insert(key).second) {
      if (message != nullptr) {
        *message = "duplicate scenario task id: " + key;
      }
      return false;
    }
  }
  std::unordered_set<std::string> workflow_ids;
  for (const auto& workflow : catalog.workflows) {
    if (workflow.scenario_id.empty() || workflow.workflow_id.empty()) {
      if (message != nullptr) {
        *message = "scenario workflow id is empty";
      }
      return false;
    }
    if (workflow.task_ids.empty()) {
      if (message != nullptr) {
        *message = "scenario workflow has no tasks: " + workflow.scenario_id + "/" +
                   workflow.workflow_id;
      }
      return false;
    }
    const auto key = workflow.scenario_id + "/" + workflow.workflow_id;
    if (!workflow_ids.insert(key).second) {
      if (message != nullptr) {
        *message = "duplicate scenario workflow id: " + key;
      }
      return false;
    }
    for (const auto& task_id : workflow.task_ids) {
      if (FindScenarioTask(catalog, workflow.scenario_id, task_id) == nullptr) {
        if (message != nullptr) {
          *message = "scenario workflow references unknown task: " + key + "/" + task_id;
        }
        return false;
      }
    }
  }
  if (message != nullptr) {
    *message = "scenario catalog ok";
  }
  return true;
}

ScenarioWorkflowSubmitDecision PlanScenarioWorkflowSubmit(
    const ScenarioWorkflowSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message) {
  ScenarioWorkflowSubmitDecision decision;
  decision.scenario_id = request.scenario_id;
  decision.workflow_id = request.workflow_id;

  if (catalog == nullptr) {
    decision.message = catalog_error_message;
    return decision;
  }

  const auto* workflow = FindScenarioWorkflow(*catalog, request.scenario_id, request.workflow_id);
  if (workflow == nullptr) {
    decision.message =
        "unknown scenario workflow: " + request.scenario_id + "/" + request.workflow_id;
    return decision;
  }

  const auto base_order_id =
      request.order_id.empty()
          ? "workflow_" + request.scenario_id + "_" + request.workflow_id
          : request.order_id;
  const auto priority = request.priority > 0 ? request.priority : workflow->default_priority;

  decision.success = true;
  decision.steps.reserve(workflow->task_ids.size());
  for (std::size_t index = 0; index < workflow->task_ids.size(); ++index) {
    ScenarioWorkflowStepPlan step;
    step.scenario_id = request.scenario_id;
    step.task_id = workflow->task_ids[index];
    step.order_id = base_order_id + "_step_" +
                    std::to_string(static_cast<int>(index) + 1);
    step.priority = priority - static_cast<int>(index);
    step.start_if_idle = request.start_if_idle && index == 0U;
    step.preempt_current = request.preempt_current && index == 0U;
    decision.steps.push_back(step);
  }
  decision.message = "planned scenario workflow " + request.scenario_id + "/" +
                     request.workflow_id + " with " +
                     std::to_string(decision.steps.size()) + " task(s)";
  return decision;
}

ScenarioTaskSubmitDecision PlanScenarioTaskSubmit(
    const ScenarioTaskSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message) {
  ScenarioTaskSubmitDecision decision;
  decision.scenario_id = request.scenario_id;
  decision.task_id = request.task_id;
  decision.start_if_idle = request.start_if_idle;
  decision.preempt_current = request.preempt_current;

  if (catalog == nullptr) {
    decision.message = catalog_error_message;
    return decision;
  }

  const auto* task = FindScenarioTask(*catalog, request.scenario_id, request.task_id);
  if (task == nullptr) {
    decision.message = "unknown scenario task: " + request.scenario_id + "/" + request.task_id;
    return decision;
  }

  decision.success = true;
  decision.order_id =
      request.order_id.empty()
          ? "scenario_" + request.scenario_id + "_" + request.task_id
          : request.order_id;
  decision.priority = request.priority > 0 ? request.priority : task->default_priority;
  const auto task_type = task->task_type.empty() ? "station_transport" : task->task_type;
  if (task_type == "facility_action") {
    decision.dispatch_kind = ScenarioTaskDispatchKind::kFacilityAction;
    decision.resource_id = task->resource_id;
    decision.resource_type = task->resource_type;
    decision.action = task->action;
    decision.hold_after_action = task->hold_after_action;
    decision.message =
        "planned scenario facility task " + request.scenario_id + "/" + request.task_id;
    return decision;
  }
  if (task_type == "station_sequence") {
    decision.dispatch_kind = ScenarioTaskDispatchKind::kStationSequence;
    decision.station_ids = task->station_sequence_ids;
    decision.message =
        "planned scenario station sequence task " + request.scenario_id + "/" +
        request.task_id;
    return decision;
  }
  if (task_type != "station_transport") {
    decision.success = false;
    decision.message =
        "unsupported scenario task type: " + request.scenario_id + "/" +
        request.task_id + "/" + task_type;
    return decision;
  }

  decision.dispatch_kind = ScenarioTaskDispatchKind::kFleetStation;
  decision.required_capability = task->required_capability;
  decision.pickup_station = task->pickup_station_id;
  decision.dropoff_station = task->dropoff_station_id;
  decision.message = "planned scenario task " + request.scenario_id + "/" + request.task_id;
  return decision;
}

}  // namespace robot_tasks
