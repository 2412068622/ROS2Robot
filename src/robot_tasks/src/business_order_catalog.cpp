#include "robot_tasks/business_order_catalog.hpp"

#include <fstream>

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

}  // namespace

std::optional<BusinessOrderCatalog> LoadBusinessOrderCatalog(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return std::nullopt;
  }

  BusinessOrderCatalog catalog;
  BusinessOrderType current;
  bool in_entry = false;
  bool has_type = false;
  bool has_scenario = false;
  bool has_workflow = false;

  auto flush = [&]() {
    if (in_entry && has_type && has_scenario && has_workflow) {
      catalog.order_types.push_back(current);
    }
    current = BusinessOrderType{};
    in_entry = false;
    has_type = false;
    has_scenario = false;
    has_workflow = false;
  };

  std::string line;
  while (std::getline(input, line)) {
    line = Trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (line == "business_orders:" || line == "order_types:") {
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      flush();
      in_entry = true;
      line = Trim(line.substr(2));
    }
    if (!in_entry) {
      continue;
    }
    if (auto value = ValueAfterColon(line, "business_type")) {
      current.business_type = *value;
      has_type = !current.business_type.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "label")) {
      current.label = *value;
      continue;
    }
    if (auto value = ValueAfterColon(line, "scenario_id")) {
      current.scenario_id = *value;
      has_scenario = !current.scenario_id.empty();
      continue;
    }
    if (auto value = ValueAfterColon(line, "workflow_id")) {
      current.workflow_id = *value;
      has_workflow = !current.workflow_id.empty();
      continue;
    }
  }
  flush();
  return catalog;
}

const BusinessOrderType* FindBusinessOrderType(
    const BusinessOrderCatalog& catalog, const std::string& business_type) {
  for (const auto& order_type : catalog.order_types) {
    if (order_type.business_type == business_type) {
      return &order_type;
    }
  }
  return nullptr;
}

BusinessOrderSubmitDecision PlanBusinessOrderSubmit(
    const BusinessOrderSubmitRequest& request,
    const BusinessOrderCatalog* catalog,
    const std::string& catalog_error_message) {
  BusinessOrderSubmitDecision decision;
  decision.business_order_id = request.business_order_id;
  decision.business_type = request.business_type;
  decision.priority = request.priority;
  decision.start_if_idle = request.start_if_idle;
  decision.preempt_current = request.preempt_current;

  if (request.business_order_id.empty() || request.business_type.empty()) {
    decision.message = "business_order_id or business_type is empty";
    return decision;
  }
  if (catalog == nullptr) {
    decision.message = catalog_error_message;
    return decision;
  }
  const auto* order_type = FindBusinessOrderType(*catalog, request.business_type);
  if (order_type == nullptr) {
    decision.message = "unsupported business_type: " + request.business_type;
    return decision;
  }

  decision.success = true;
  decision.routed_scenario_id = order_type->scenario_id;
  decision.routed_workflow_id = order_type->workflow_id;
  decision.workflow_order_id = "business_" + request.business_order_id;
  decision.message = "accepted business order " + request.business_order_id + " as " +
                     order_type->scenario_id + "/" + order_type->workflow_id;
  return decision;
}

}  // namespace robot_tasks
