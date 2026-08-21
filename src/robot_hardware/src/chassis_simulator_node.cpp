#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <optional>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/chassis_kinematics.hpp"
#include "robot_hardware/chassis_packet.hpp"

using namespace std::chrono_literals;

namespace {

constexpr double kReceiveBufferTimeoutSeconds = 0.5;

std::optional<sockaddr_in> MakeBindAddress(const std::string& host, const int port) {
  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    return std::nullopt;
  }
  return addr;
}

}  // namespace

class ChassisSimulatorNode final : public rclcpp::Node {
 public:
  ChassisSimulatorNode() : Node("chassis_simulator_node") {
    bind_host_ = declare_parameter<std::string>("bind_host", "0.0.0.0");
    port_ = declare_parameter<int>("port", 9000);
    battery_voltage_ = declare_parameter<double>("battery_voltage", 24.0);
    OpenSocket();
    last_update_ = now();
    last_rx_buffer_update_ = last_update_;
    timer_ = create_wall_timer(20ms, [this]() { Tick(); });
  }

  ~ChassisSimulatorNode() override {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

 private:
  void OpenSocket() {
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create simulator UDP socket: %s", std::strerror(errno));
      return;
    }

    const int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const auto addr = MakeBindAddress(bind_host_, port_);
    if (!addr.has_value()) {
      RCLCPP_ERROR(get_logger(), "invalid simulator bind_host '%s'", bind_host_.c_str());
      close(fd_);
      fd_ = -1;
      return;
    }
    if (bind(fd_, reinterpret_cast<const sockaddr*>(&*addr), sizeof(*addr)) != 0) {
      RCLCPP_ERROR(
          get_logger(), "failed to bind simulator UDP socket on %s:%d: %s",
          bind_host_.c_str(), port_, std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return;
    }
    RCLCPP_INFO(get_logger(), "UDP chassis simulator listening on %s:%d", bind_host_.c_str(), port_);
  }

  void Tick() {
    const auto stamp = now();
    const double dt = std::max(0.001, (stamp - last_update_).seconds());
    last_update_ = stamp;

    ReadCommands(stamp);
    pose_ = robot_hardware::IntegratePlanarMotion(pose_, command_, dt);
    PublishPackets();
  }

  void ReadCommands(const rclcpp::Time& stamp) {
    if (fd_ < 0) {
      return;
    }
    if (!rx_buffer_.empty() &&
        (stamp - last_rx_buffer_update_).seconds() > kReceiveBufferTimeoutSeconds) {
      rx_buffer_.clear();
      RCLCPP_WARN(get_logger(), "cleared stale simulator receive buffer");
    }
    char buffer[512];
    sockaddr_in from {};
    socklen_t from_len = sizeof(from);
    const ssize_t received =
        recvfrom(fd_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
    if (received < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "simulator UDP receive failed: %s",
            std::strerror(errno));
      }
      return;
    }
    if (received == 0) {
      return;
    }

    remote_addr_ = from;
    has_remote_ = true;
    rx_buffer_.append(buffer, static_cast<std::size_t>(received));
    last_rx_buffer_update_ = stamp;
    std::size_t newline = std::string::npos;
    while ((newline = rx_buffer_.find('\n')) != std::string::npos) {
      const std::string line = rx_buffer_.substr(0, newline);
      rx_buffer_.erase(0, newline + 1);
      if (auto command = robot_hardware::ParseCommandLine(line)) {
        command_ = *command;
      }
    }
    if (rx_buffer_.size() > 1024) {
      rx_buffer_.clear();
      RCLCPP_WARN(get_logger(), "cleared oversized simulator receive buffer");
    }
  }

  void PublishPackets() {
    if (fd_ < 0 || !has_remote_) {
      return;
    }
    robot_hardware::ChassisOdometryPacket odom;
    odom.x_m = pose_.x_m;
    odom.y_m = pose_.y_m;
    odom.yaw_rad = pose_.yaw_rad;
    odom.linear_x_mps = command_.linear_x_mps;
    odom.angular_z_radps = command_.angular_z_radps;
    odom.battery_voltage = battery_voltage_;
    robot_hardware::ChassisStatePacket state;
    state.status = "simulated";
    state.battery_voltage = battery_voltage_;
    const std::string packet = robot_hardware::EncodeOdometry(odom) + robot_hardware::EncodeState(state);
    const ssize_t sent =
        sendto(fd_, packet.data(), packet.size(), 0,
               reinterpret_cast<const sockaddr*>(&remote_addr_), sizeof(remote_addr_));
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "simulator UDP send failed: %s", std::strerror(errno));
    }
  }

  std::string bind_host_;
  int port_ = 9000;
  int fd_ = -1;
  sockaddr_in remote_addr_ {};
  bool has_remote_ = false;
  std::string rx_buffer_;
  robot_hardware::PlanarPose pose_;
  double battery_voltage_ = 24.0;
  robot_hardware::ChassisCommand command_;
  rclcpp::Time last_update_;
  rclcpp::Time last_rx_buffer_update_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
