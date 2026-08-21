#pragma once

#include <string>

#include "robot_tasks/mission_preflight.hpp"

namespace robot_tasks {

struct OpportunityChargingPlanRequest {
  double requested_battery_voltage = 0.0;
  double current_battery_voltage = 0.0;
  double requested_opportunity_threshold_voltage = 0.0;
  double default_opportunity_threshold_voltage = 0.0;
  double requested_critical_threshold_voltage = 0.0;
  double default_critical_threshold_voltage = 0.0;
  bool requested_require_idle_queue = false;
  bool default_require_idle_queue = false;
  bool mission_active = false;
  bool queue_empty = true;
  int requested_priority = 0;
  int default_priority = 0;
  bool requested_preempt_current = false;
};

struct OpportunityChargingDecision {
  bool success = true;
  bool queue_charging = false;
  bool critical_battery = false;
  bool preempt_current = false;
  bool require_idle_queue = false;
  int priority = 0;
  double evaluated_battery_voltage = 0.0;
  double opportunity_threshold_voltage = 0.0;
  double critical_threshold_voltage = 0.0;
  std::string branch = "skip";
  std::string message;
  std::string queued_message_prefix = "opportunity ";
};

struct LowBatteryDockRequest {
  bool enabled = false;
  double battery_voltage = 0.0;
  double threshold_voltage = 0.0;
  bool request_already_sent = false;
};

struct LowBatteryDockDecision {
  bool low_battery_request_sent = false;
  bool request_dock_return = false;
  std::string reason;
  std::string branch = "disabled";
};

struct ChargingRequestGateRequest {
  std::string mission_id;
  std::string charger_id;
  std::string charger_station_id;
  bool station_catalog_loaded = true;
  std::string station_catalog_error_message;
  bool charger_station_known = true;
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

struct ChargingRequestGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string charger_id;
  std::string message;
};

OpportunityChargingDecision PlanOpportunityCharging(
    const OpportunityChargingPlanRequest& request);

LowBatteryDockDecision PlanLowBatteryDock(const LowBatteryDockRequest& request);

ChargingRequestGateDecision PlanChargingRequestGate(
    const ChargingRequestGateRequest& request);

}  // namespace robot_tasks
