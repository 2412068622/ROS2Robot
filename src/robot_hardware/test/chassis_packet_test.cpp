#include "robot_hardware/chassis_packet.hpp"
#include "robot_hardware/chassis_kinematics.hpp"

#include <gtest/gtest.h>

namespace robot_hardware {

namespace {
constexpr double kPi = 3.14159265358979323846264338327950288;
}  // namespace

TEST(ChassisPacketTest, EncodeCommand_ValidCommand_FormatsLineProtocol) {
  ChassisCommand command;
  command.linear_x_mps = 0.3;
  command.linear_y_mps = 0.1;
  command.angular_z_radps = -0.2;

  EXPECT_EQ(EncodeCommand(command), "CMD 0.3000 0.1000 -0.2000\n");
}

TEST(ChassisPacketTest, ParseCommandLine_ValidCommand_ReturnsCommand) {
  const auto command = ParseCommandLine("CMD 0.125 -0.500");

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear_x_mps, 0.125);
  EXPECT_DOUBLE_EQ(command->linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(command->angular_z_radps, -0.5);
}

TEST(ChassisPacketTest, ParseCommandLine_ThreeAxisCommand_ReturnsHolonomicCommand) {
  const auto command = ParseCommandLine("CMD 0.125 -0.250 -0.500");

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear_x_mps, 0.125);
  EXPECT_DOUBLE_EQ(command->linear_y_mps, -0.250);
  EXPECT_DOUBLE_EQ(command->angular_z_radps, -0.5);
}

TEST(ChassisPacketTest, ParsePacketLine_ValidOdometry_ReturnsOdometryPacket) {
  const auto packet = ParsePacketLine("ODOM 1.25 0.10 0.02 0.30 -0.04 24.1");

  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->kind, ChassisPacketKind::kOdometry);
  EXPECT_DOUBLE_EQ(packet->odometry.x_m, 1.25);
  EXPECT_DOUBLE_EQ(packet->odometry.y_m, 0.10);
  EXPECT_DOUBLE_EQ(packet->odometry.yaw_rad, 0.02);
  EXPECT_DOUBLE_EQ(packet->odometry.linear_x_mps, 0.30);
  EXPECT_DOUBLE_EQ(packet->odometry.angular_z_radps, -0.04);
  ASSERT_TRUE(packet->odometry.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->odometry.battery_voltage, 24.1);
}

TEST(ChassisPacketTest, ParsePacketLine_ValidState_ReturnsStatePacket) {
  const auto packet = ParsePacketLine("STATE running 23.8");

  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->kind, ChassisPacketKind::kState);
  EXPECT_EQ(packet->state.status, "running");
  ASSERT_TRUE(packet->state.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->state.battery_voltage, 23.8);
}

TEST(ChassisPacketTest, ParsePacketLine_MalformedPacket_ReturnsNullopt) {
  EXPECT_FALSE(ParsePacketLine("ODOM 1.0 2.0 bad").has_value());
}

TEST(ChassisPacketTest, ChassisPacketStream_PartialAndStickyLines_DrainsCompletePackets) {
  ChassisPacketStream stream;
  stream.Append("ODOM 1 2 0.1 0.3 0.0 24");
  EXPECT_TRUE(stream.DrainPackets().empty());
  stream.Append("\nSTATE running 24.0\n");

  const auto packets = stream.DrainPackets();

  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kOdometry);
  EXPECT_EQ(packets[1].kind, ChassisPacketKind::kState);
}

TEST(ChassisPacketTest, ChassisPacketStream_BufferOverflow_ClearsBufferAndReportsOverflow) {
  ChassisPacketStream stream(8);

  stream.Append("0123456789");

  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());
}

TEST(ChassisPacketTest, ChassisPacketStream_MalformedCompleteLine_CountsInvalidLine) {
  ChassisPacketStream stream;

  stream.Append("NOPE bad packet\n");
  const auto packets = stream.DrainPackets();

  EXPECT_TRUE(packets.empty());
  EXPECT_EQ(stream.InvalidLineCount(), 1U);
}

TEST(ChassisPacketTest, MickBinaryPacketStream_StickyFrames_DrainsPackets) {
  robot_hardware::ChassisOdometryPacket odom;
  odom.x_m = 1.2;
  odom.y_m = -0.3;
  odom.yaw_rad = 0.1;
  odom.linear_x_mps = 0.4;
  odom.angular_z_radps = -0.2;
  odom.battery_voltage = 24.5;
  std::vector<uint8_t> payload;
  auto append = [&payload](const int value) {
    payload.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    payload.push_back(static_cast<uint8_t>(value & 0xff));
  };
  append(1200);
  append(-300);
  append(100);
  append(400);
  append(-200);
  append(2450);

  const auto frame = EncodeMickBinaryFrame(0xa1, payload);
  MickBinaryPacketStream stream;
  stream.Append(std::string(reinterpret_cast<const char*>(frame.data()), frame.size()));

  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].kind, ChassisPacketKind::kOdometry);
  EXPECT_NEAR(packets[0].odometry.x_m, 1.2, 1e-6);
  EXPECT_NEAR(packets[0].odometry.y_m, -0.3, 1e-6);
  ASSERT_TRUE(packets[0].odometry.battery_voltage.has_value());
  EXPECT_NEAR(*packets[0].odometry.battery_voltage, 24.5, 1e-6);
}

TEST(ChassisPacketTest, MickBinaryPacketStream_BadChecksum_CountsInvalidFrame) {
  auto frame = EncodeMickSpeedCommand(ChassisCommand{});
  frame[frame.size() - 3] ^= 0x55;
  MickBinaryPacketStream stream;
  stream.Append(std::string(reinterpret_cast<const char*>(frame.data()), frame.size()));

  EXPECT_TRUE(stream.DrainPackets().empty());
  EXPECT_EQ(stream.InvalidFrameCount(), 1U);
}

TEST(ChassisKinematicsTest, IntegratePlanarMotion_HolonomicCommand_UsesBodyYVelocity) {
  ChassisCommand command;
  command.linear_x_mps = 1.0;
  command.linear_y_mps = 0.5;
  command.angular_z_radps = 0.1;

  const auto pose = IntegratePlanarMotion(PlanarPose{}, command, 2.0);

  EXPECT_NEAR(pose.x_m, 2.0, 1e-6);
  EXPECT_NEAR(pose.y_m, 1.0, 1e-6);
  EXPECT_NEAR(pose.yaw_rad, 0.2, 1e-6);
}

TEST(ChassisKinematicsTest, IntegratePlanarMotion_NormalizesYaw) {
  ChassisCommand command;
  command.angular_z_radps = 1.0;
  PlanarPose start;
  start.yaw_rad = kPi - 0.05;

  const auto pose = IntegratePlanarMotion(start, command, 0.2);

  EXPECT_GT(pose.yaw_rad, -kPi);
  EXPECT_LT(pose.yaw_rad, kPi);
  EXPECT_NEAR(pose.yaw_rad, -kPi + 0.15, 1e-9);
}

TEST(ChassisKinematicsTest, EstimateWheelSpeeds_MecanumCommand_ProducesFourWheelProfile) {
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.2;
  command.angular_z_radps = 0.4;

  const auto speeds = EstimateWheelSpeeds(command, "mecanum", 0.15, 0.42, 0.36);

  EXPECT_NE(speeds.rpm[0], speeds.rpm[1]);
  EXPECT_NE(speeds.rpm[2], speeds.rpm[3]);
}

TEST(ChassisKinematicsTest, NormalizeKinematicsModel_UnsupportedSteeringModesDegradeToDiffDrive) {
  EXPECT_EQ(NormalizeKinematicsModel("ackermann"), "diff_drive");
  EXPECT_EQ(NormalizeKinematicsModel("acker"), "diff_drive");
  EXPECT_EQ(NormalizeKinematicsModel("four_ws4wd"), "diff_drive");
  EXPECT_EQ(NormalizeKinematicsModel("4ws4wd"), "diff_drive");
  EXPECT_EQ(NormalizeKinematicsModel("four_wheel_steer"), "diff_drive");
}

TEST(ChassisKinematicsTest, EstimateWheelSpeeds_FourWheelSteerAliasUsesDiffDriveProfile) {
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.2;
  command.angular_z_radps = 0.4;

  const auto degraded = EstimateWheelSpeeds(command, "four_ws4wd", 0.15, 0.42, 0.36);
  const auto diff_drive = EstimateWheelSpeeds(command, "diff_drive", 0.15, 0.42, 0.36);

  EXPECT_EQ(degraded.rpm, diff_drive.rpm);
}

TEST(ChassisKinematicsTest, EstimateCommandFromWheelSpeeds_DiffDriveRoundTrip) {
  ChassisCommand command;
  command.linear_x_mps = 0.45;
  command.angular_z_radps = -0.3;

  const auto speeds = EstimateWheelSpeeds(command, "diff_drive", 0.15, 0.42, 0.36);
  const auto estimated = EstimateCommandFromWheelSpeeds(speeds, "diff_drive", 0.15, 0.42, 0.36);

  EXPECT_NEAR(estimated.linear_x_mps, command.linear_x_mps, 1e-9);
  EXPECT_NEAR(estimated.angular_z_radps, command.angular_z_radps, 1e-9);
  EXPECT_NEAR(estimated.linear_y_mps, 0.0, 1e-9);
}

TEST(ChassisKinematicsTest, EstimateCommandFromWheelSpeeds_MecanumRoundTrip) {
  ChassisCommand command;
  command.linear_x_mps = 0.45;
  command.linear_y_mps = -0.2;
  command.angular_z_radps = 0.3;

  const auto speeds = EstimateWheelSpeeds(command, "mecanum", 0.15, 0.42, 0.36);
  const auto estimated = EstimateCommandFromWheelSpeeds(speeds, "mecanum", 0.15, 0.42, 0.36);

  EXPECT_NEAR(estimated.linear_x_mps, command.linear_x_mps, 1e-9);
  EXPECT_NEAR(estimated.linear_y_mps, command.linear_y_mps, 1e-9);
  EXPECT_NEAR(estimated.angular_z_radps, command.angular_z_radps, 1e-9);
}

}  // namespace robot_hardware
