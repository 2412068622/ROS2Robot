#pragma once

#include <memory>
#include <string>

#include "robot_hardware/chassis_backend.hpp"
#include "robot_hardware/chassis_kinematics.hpp"
#include "robot_hardware/chassis_packet.hpp"

namespace robot_hardware {

struct ChassisSystemAdapterConfig {
  std::string calibration_profile = "nominal_reference";
  std::string kinematics_model = "diff_drive";
  std::string protocol = "text";
  double wheel_diameter_m = 0.15;
  double wheel_base_m = 0.42;
  double track_width_m = 0.43;
  double left_encoder_scale = 1.0;
  double right_encoder_scale = 1.0;
  int left_direction_sign = 1;
  int right_direction_sign = 1;
  double fallback_battery_voltage = 24.0;
};

bool ValidateChassisSystemAdapterConfig(
    const ChassisSystemAdapterConfig& config, std::string* error);

struct ChassisSystemState {
  std::string backend_name;
  std::string hardware_status = "running";
  bool backend_open = false;
  bool connected = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  double battery_voltage = 24.0;
  WheelSpeeds wheel_speeds;
  unsigned long long packet_sequence = 0;
};

class ChassisSystemAdapter {
 public:
  ChassisSystemAdapter(
      std::unique_ptr<ChassisBackend> backend, ChassisSystemAdapterConfig config = {});

  std::string BackendName() const;
  bool Open(std::string* error);
  void Close();
  bool IsOpen() const;
  void SetKinematicsModel(const std::string& model);
  bool Write(const ChassisCommand& command, std::string* error);
  bool Read(double period_seconds, ChassisSystemState* state, std::string* error);
  void ResetOdometry();
  const ChassisSystemState& State() const;

 private:
  ChassisCommand NormalizeCommand(const ChassisCommand& command) const;
  void IntegrateMock(double period_seconds);
  bool ReadBackendPackets(std::string* error);
  void ApplyPacket(const ChassisPacket& packet);
  void RefreshOpenState();

  std::unique_ptr<ChassisBackend> backend_;
  ChassisSystemAdapterConfig config_;
  ChassisPacketStream text_stream_;
  MickBinaryPacketStream binary_stream_;
  ChassisCommand command_;
  ChassisSystemState state_;
};

}  // namespace robot_hardware
