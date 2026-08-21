#include "robot_tasks/mission_recovery_policy.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(MissionRecoveryPolicyTest, DisabledPolicy_ReturnsNoneAndNotRecoverable) {
  MissionRecoveryConfig config;
  config.enabled = false;
  config.policy = "retry_then_dock";

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "none");
  EXPECT_FALSE(decision.recoverable);
  EXPECT_NE(decision.message.find("disabled"), std::string::npos);
}

TEST(MissionRecoveryPolicyTest, RetryThenDock_BeforeLimit_ReturnsRetry) {
  MissionRecoveryConfig config;
  config.policy = "retry_then_dock";
  config.retry_limit = 2;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "retry");
  EXPECT_TRUE(decision.recoverable);
  EXPECT_NE(decision.message.find("2/2"), std::string::npos);
}

TEST(MissionRecoveryPolicyTest, RetryThenDock_AfterLimit_ReturnsDock) {
  MissionRecoveryConfig config;
  config.policy = "retry_then_dock";
  config.retry_limit = 1;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "dock");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, RetryOnly_AfterLimit_ReturnsManual) {
  MissionRecoveryConfig config;
  config.policy = "retry_only";
  config.retry_limit = 1;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "manual");
  EXPECT_FALSE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, RetryThenManual_AfterLimit_ReturnsManual) {
  MissionRecoveryConfig config;
  config.policy = "retry_then_manual";
  config.retry_limit = 1;

  const auto decision = DecideMissionRecovery(config, 1, false, "delivery");

  EXPECT_EQ(decision.action, "manual");
  EXPECT_FALSE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, DockPolicy_ReturnsDockWithoutRetry) {
  MissionRecoveryConfig config;
  config.policy = "dock";
  config.retry_limit = 3;

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "dock");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, DockMissionFailure_ReturnsManual) {
  MissionRecoveryConfig config;
  config.policy = "retry_then_dock";
  config.retry_limit = 3;

  const auto decision = DecideMissionRecovery(config, 0, true, "dock_return");

  EXPECT_EQ(decision.action, "manual");
  EXPECT_FALSE(decision.recoverable);
  EXPECT_NE(decision.message.find("dock mission failed"), std::string::npos);
}

TEST(MissionRecoveryPolicyTest, NegativeRetryLimit_ClampsToZeroAndUsesFallback) {
  MissionRecoveryConfig config;
  config.policy = "retry_then_dock";
  config.retry_limit = -2;

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "dock");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, PolicyName_IsCaseInsensitive) {
  MissionRecoveryConfig config;
  config.policy = "ReTrY_ThEn_DoCk";
  config.retry_limit = 1;

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "retry");
  EXPECT_TRUE(decision.recoverable);
}

TEST(MissionRecoveryPolicyTest, UnknownPolicy_ReturnsManual) {
  MissionRecoveryConfig config;
  config.policy = "call_operator_later";

  const auto decision = DecideMissionRecovery(config, 0, false, "delivery");

  EXPECT_EQ(decision.action, "manual");
  EXPECT_FALSE(decision.recoverable);
  EXPECT_NE(decision.message.find("unknown recovery policy"), std::string::npos);
}

}  // namespace
}  // namespace robot_tasks
