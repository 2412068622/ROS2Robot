#include "robot_tasks/mission_recovery_policy.hpp"

#include <algorithm>
#include <cctype>

namespace robot_tasks {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool RetryPolicy(const std::string& policy) {
  return policy == "retry" || policy == "retry_only" || policy == "retry_then_dock" ||
         policy == "retry_then_manual";
}

MissionRecoveryDecision Decision(
    const std::string& action, const std::string& message, const bool recoverable) {
  MissionRecoveryDecision decision;
  decision.action = action;
  decision.message = message;
  decision.recoverable = recoverable;
  return decision;
}

}  // namespace

MissionRecoveryDecision DecideMissionRecovery(
    const MissionRecoveryConfig& config, const int retry_count, const bool is_dock_mission,
    const std::string& mission_id) {
  const auto policy = ToLower(config.policy);
  if (!config.enabled || policy == "disabled" || policy == "none") {
    return Decision("none", "failure recovery disabled for " + mission_id, false);
  }
  if (is_dock_mission) {
    return Decision("manual", "dock mission failed, operator intervention required", false);
  }
  if (RetryPolicy(policy) && retry_count < std::max(0, config.retry_limit)) {
    return Decision(
        "retry", "retrying failed mission " + mission_id + " (" +
                     std::to_string(retry_count + 1) + "/" +
                     std::to_string(std::max(0, config.retry_limit)) + ")",
        true);
  }
  if (policy == "retry_then_dock" || policy == "dock") {
    return Decision("dock", "recovery will return to dock after failure of " + mission_id, true);
  }
  if (policy == "retry_then_manual" || policy == "manual" || policy == "retry_only") {
    return Decision("manual", "operator intervention required after failure of " + mission_id, false);
  }
  return Decision("manual", "unknown recovery policy " + config.policy + ", operator intervention required", false);
}

}  // namespace robot_tasks
