#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

struct BusinessOrderType {
  std::string business_type;
  std::string label;
  std::string scenario_id;
  std::string workflow_id;
};

struct BusinessOrderCatalog {
  std::vector<BusinessOrderType> order_types;
};

struct BusinessOrderSubmitRequest {
  std::string business_order_id;
  std::string business_type;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

struct BusinessOrderSubmitDecision {
  bool success{false};
  std::string business_order_id;
  std::string business_type;
  std::string routed_service{"/submit_scenario_workflow"};
  std::string routed_scenario_id;
  std::string routed_workflow_id;
  std::string workflow_order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string message;
};

std::optional<BusinessOrderCatalog> LoadBusinessOrderCatalog(const std::filesystem::path& path);
const BusinessOrderType* FindBusinessOrderType(
    const BusinessOrderCatalog& catalog, const std::string& business_type);
BusinessOrderSubmitDecision PlanBusinessOrderSubmit(
    const BusinessOrderSubmitRequest& request,
    const BusinessOrderCatalog* catalog,
    const std::string& catalog_error_message);

}  // namespace robot_tasks
