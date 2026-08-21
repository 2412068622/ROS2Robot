#include "robot_teleop/cmd_vel_safety.hpp"

#include <gtest/gtest.h>

namespace {

TEST(CmdVelSafetyTest, ScalesAngularVelocityWhenLinearSpeedIsScaled) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 0.8;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.25, 1.0);

  EXPECT_NEAR(limited.linear.x, 0.25, 1e-9);
  EXPECT_NEAR(limited.angular.z, 0.2, 1e-9);
}

TEST(CmdVelSafetyTest, ClampsPureRotationInDynamicSpeedLimitedZone) {
  geometry_msgs::msg::Twist twist;
  twist.angular.z = 1.4;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.25, 0.5);

  EXPECT_NEAR(limited.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(limited.angular.z, 0.5, 1e-9);
}

TEST(CmdVelSafetyTest, LeavesTwistUnchangedWhenLimitIsDisabled) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 1.4;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.0, 0.5);

  EXPECT_NEAR(limited.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(limited.angular.z, 1.4, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_EmergencyStop_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_twist.angular.z = 0.2;
  request.emergency_stop = true;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kEmergencyStop);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(decision.output_twist.angular.z, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_RuntimeSafetyStop_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.runtime_safety_stop = true;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kRuntimeSafetyStop);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_UnavailableInput_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_available = false;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kInputUnavailable);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_StaleInput_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_fresh = false;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kInputStale);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_FreshInput_AppliesDynamicLimit) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 1.0;
  request.input_twist.angular.z = 0.8;
  request.dynamic_speed_limit_mps = 0.5;
  request.dynamic_angular_limit_radps = 0.6;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_FALSE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kNone);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(decision.output_twist.angular.z, 0.4, 1e-9);
}

TEST(CmdVelSafetyTest, SelectCommandSource_ManualTakeover_ChoosesTeleop) {
  EXPECT_EQ(robot_teleop::SelectCommandSource(true, "nav2"), "teleop");
  EXPECT_EQ(robot_teleop::SelectCommandSource(false, "nav2"), "nav2");
}

TEST(CmdVelSafetyTest, DescribeCommandState_ReportsHighestPriorityState) {
  EXPECT_EQ(robot_teleop::DescribeCommandState(true, true, true, "nav2"), "emergency_stop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, true, true, "nav2"), "safety_stop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, false, true, "nav2"),
            "manual_takeover:teleop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, false, false, "nav2"), "nav2");
}

}  // namespace
