#include "robot_tasks/mission_charging_workflow.hpp"

#include <string>

namespace robot_tasks {
namespace {

double EffectivePositiveValue(const double requested, const double fallback) {
  return requested > 0.0 ? requested : fallback;
}

std::string VoltageString(const double voltage) {
  return std::to_string(voltage) + "V";
}

}  // namespace

OpportunityChargingDecision PlanOpportunityCharging(
    const OpportunityChargingPlanRequest& request) {
  OpportunityChargingDecision decision;
  decision.evaluated_battery_voltage =
      EffectivePositiveValue(request.requested_battery_voltage, request.current_battery_voltage);
  decision.opportunity_threshold_voltage = EffectivePositiveValue(
      request.requested_opportunity_threshold_voltage,
      request.default_opportunity_threshold_voltage);
  decision.critical_threshold_voltage = EffectivePositiveValue(
      request.requested_critical_threshold_voltage, request.default_critical_threshold_voltage);
  decision.require_idle_queue =
      request.requested_require_idle_queue || request.default_require_idle_queue;
  decision.priority =
      request.requested_priority > 0 ? request.requested_priority : request.default_priority;

  if (decision.evaluated_battery_voltage <= 0.0) {
    decision.success = false;
    decision.branch = "battery_unavailable";
    decision.message = "battery voltage unavailable for opportunity charging";
    return decision;
  }

  if (decision.evaluated_battery_voltage > decision.opportunity_threshold_voltage) {
    decision.branch = "above_threshold";
    decision.message = "opportunity charging skipped: battery above threshold " +
                       VoltageString(decision.opportunity_threshold_voltage);
    return decision;
  }

  decision.critical_battery =
      decision.evaluated_battery_voltage <= decision.critical_threshold_voltage;
  if (decision.require_idle_queue && !decision.critical_battery &&
      (request.mission_active || !request.queue_empty)) {
    decision.branch = "busy_noncritical";
    decision.message =
        "opportunity charging skipped: mission runner is busy and battery is not critical";
    return decision;
  }

  decision.branch = decision.critical_battery ? "critical_charge" : "opportunity_charge";
  decision.queue_charging = true;
  decision.preempt_current = request.requested_preempt_current || decision.critical_battery;
  decision.queued_message_prefix = decision.critical_battery ? "critical " : "opportunity ";
  return decision;
}

LowBatteryDockDecision PlanLowBatteryDock(const LowBatteryDockRequest& request) {
  LowBatteryDockDecision decision;
  decision.low_battery_request_sent = request.request_already_sent;

  if (!request.enabled) {
    decision.branch = "disabled";
    return decision;
  }
  if (request.battery_voltage <= 0.0) {
    decision.branch = "battery_unavailable";
    return decision;
  }
  if (request.battery_voltage > request.threshold_voltage) {
    decision.branch = "above_threshold";
    decision.low_battery_request_sent = false;
    return decision;
  }
  if (request.request_already_sent) {
    decision.branch = "already_requested";
    return decision;
  }

  decision.branch = "request_dock_return";
  decision.low_battery_request_sent = true;
  decision.request_dock_return = true;
  decision.reason = "low battery " + VoltageString(request.battery_voltage);
  return decision;
}

ChargingRequestGateDecision PlanChargingRequestGate(
    const ChargingRequestGateRequest& request) {
  ChargingRequestGateDecision decision;
  decision.mission_id = request.mission_id;
  decision.charger_id = request.charger_id;

  if (!request.station_catalog_loaded) {
    decision.message = request.station_catalog_error_message.empty()
                           ? "failed to load station catalog"
                           : request.station_catalog_error_message;
    return decision;
  }
  if (!request.charger_station_known) {
    decision.message = "charger station not found: " + request.charger_station_id;
    return decision;
  }
  if (request.mission_already_active_or_queued) {
    decision.message = "charging mission already active or queued: " + request.mission_id;
    return decision;
  }
  if (!request.preflight.allowed) {
    decision.message = "charging mission preflight rejected: " + request.preflight.message;
    return decision;
  }

  decision.accepted = true;
  return decision;
}

}  // namespace robot_tasks
