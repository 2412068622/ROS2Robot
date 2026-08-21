#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_hardware {

struct ChassisCommand {
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
};

struct ChassisOdometryPacket {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double linear_x_mps = 0.0;
  double angular_z_radps = 0.0;
  std::optional<double> battery_voltage;
};

struct ChassisStatePacket {
  std::string status;
  std::optional<double> battery_voltage;
};

enum class ChassisPacketKind { kOdometry, kState };

struct ChassisPacket {
  ChassisPacketKind kind = ChassisPacketKind::kState;
  ChassisOdometryPacket odometry;
  ChassisStatePacket state;
};

std::string EncodeCommand(const ChassisCommand& command);
std::string EncodeOdometry(const ChassisOdometryPacket& odometry);
std::string EncodeState(const ChassisStatePacket& state);
std::optional<ChassisCommand> ParseCommandLine(const std::string& line);
std::optional<ChassisPacket> ParsePacketLine(const std::string& line);

struct MickBinaryFrame {
  uint8_t type = 0;
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> EncodeMickBinaryFrame(uint8_t type, const std::vector<uint8_t>& payload);
std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand& command);
std::vector<uint8_t> EncodeMickClearOdometryCommand();
std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame& frame);

class ChassisPacketStream {
 public:
  explicit ChassisPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidLineCount() const;
  void ClearOverflow();
  void ClearInvalidLineCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::string buffer_;
  bool overflowed_ = false;
  std::size_t invalid_line_count_ = 0;
};

class MickBinaryPacketStream {
 public:
  explicit MickBinaryPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidFrameCount() const;
  void ClearOverflow();
  void ClearInvalidFrameCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::vector<uint8_t> buffer_;
  bool overflowed_ = false;
  std::size_t invalid_frame_count_ = 0;
};

}  // namespace robot_hardware
