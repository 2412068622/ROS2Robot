#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace robot_teleop {

inline geometry_msgs::msg::Twist MakeZeroTwist() {
  geometry_msgs::msg::Twist twist;
  return twist;
}

inline geometry_msgs::msg::Twist ApplyDynamicSpeedLimit(
    geometry_msgs::msg::Twist twist, const double linear_limit_mps,
    const double angular_limit_radps) {
  if (linear_limit_mps <= 0.0) {
    return twist;
  }

  const double linear_speed = std::hypot(twist.linear.x, twist.linear.y);
  if (linear_speed > linear_limit_mps && linear_speed > 0.0) {
    const double scale = linear_limit_mps / linear_speed;
    twist.linear.x *= scale;
    twist.linear.y *= scale;
    twist.angular.z *= scale;
  }

  if (angular_limit_radps > 0.0) {
    twist.angular.z = std::clamp(twist.angular.z, -angular_limit_radps, angular_limit_radps);
  }
  return twist;
}

enum class CmdVelStopReason {
  kNone,
  kEmergencyStop,
  kRuntimeSafetyStop,
  kInputUnavailable,
  kInputStale,
};

struct CmdVelSafetyGateRequest {
  geometry_msgs::msg::Twist input_twist;
  bool emergency_stop = false;
  bool runtime_safety_stop = false;
  bool input_available = true;
  bool input_fresh = true;
  double dynamic_speed_limit_mps = 0.0;
  double dynamic_angular_limit_radps = 0.6;
};

struct CmdVelSafetyGateDecision {
  geometry_msgs::msg::Twist output_twist;
  CmdVelStopReason stop_reason = CmdVelStopReason::kNone;

  bool stopped() const { return stop_reason != CmdVelStopReason::kNone; }
};

inline std::string SelectCommandSource(const bool manual_takeover,
                                       const std::string& active_source) {
  return manual_takeover ? "teleop" : active_source;
}

inline std::string DescribeCommandState(const bool emergency_stop,
                                        const bool runtime_safety_stop,
                                        const bool manual_takeover,
                                        const std::string& active_source) {
  if (emergency_stop) {
    return "emergency_stop";
  }
  if (runtime_safety_stop) {
    return "safety_stop";
  }
  if (manual_takeover) {
    return "manual_takeover:teleop";
  }
  return active_source;
}

inline CmdVelSafetyGateDecision EvaluateCmdVelSafetyGate(
    const CmdVelSafetyGateRequest& request) {
  CmdVelSafetyGateDecision decision;
  if (request.emergency_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kEmergencyStop;
    return decision;
  }
  if (request.runtime_safety_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kRuntimeSafetyStop;
    return decision;
  }
  if (!request.input_available) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputUnavailable;
    return decision;
  }
  if (!request.input_fresh) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputStale;
    return decision;
  }

  decision.output_twist =
      ApplyDynamicSpeedLimit(request.input_twist, request.dynamic_speed_limit_mps,
                             request.dynamic_angular_limit_radps);
  decision.stop_reason = CmdVelStopReason::kNone;
  return decision;
}

}  // namespace robot_teleop
