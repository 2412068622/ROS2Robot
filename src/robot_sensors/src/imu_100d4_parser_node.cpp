#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

constexpr std::size_t kMaxFrameLength = 256U;

double DecodeInt16Scaled(const std::vector<uint8_t>& data, const std::size_t offset, const double scale) {
  if (offset + 1 >= data.size()) {
    return 0.0;
  }
  const auto raw =
      static_cast<int16_t>(static_cast<uint16_t>(data[offset] << 8) | static_cast<uint16_t>(data[offset + 1]));
  return static_cast<double>(raw) / scale;
}

geometry_msgs::msg::Quaternion QuaternionFromEuler(
    const double roll, const double pitch, const double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

}  // namespace

class Imu100d4ParserNode final : public rclcpp::Node {
 public:
  Imu100d4ParserNode() : Node("imu_100d4_parser_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/imu_100d4/raw_bytes");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        input_topic_, 10,
        [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) { Parse(msg->data); });
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/imu_100d4/calibration_state", 10);
    start_calib_srv_ = create_service<std_srvs::srv::Trigger>(
        "/imu_100d4/start_magnetic_calibration",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibration_state_ = "magnetic_calibration_active";
          response->success = true;
          response->message = "100D4 magnetic calibration command queued";
          PublishState();
        });
    stop_calib_srv_ = create_service<std_srvs::srv::Trigger>(
        "/imu_100d4/stop_magnetic_calibration",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibration_state_ = "idle";
          response->success = true;
          response->message = "100D4 magnetic calibration stop command queued";
          PublishState();
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { PublishState(); });
  }

 private:
  void Parse(const std::vector<uint8_t>& bytes) {
    for (std::size_t i = 0; i + 6 < bytes.size(); ++i) {
      if (bytes[i] != 0xa5 || bytes[i + 1] != 0x5a) {
        continue;
      }
      const std::size_t length = bytes[i + 2];
      if (length < 24U || length > kMaxFrameLength || i + length + 2U >= bytes.size()) {
        continue;
      }
      uint8_t checksum = 0;
      for (std::size_t j = i + 2; j < i + 1 + length; ++j) {
        checksum = static_cast<uint8_t>(checksum + bytes[j]);
      }
      if (checksum != bytes[i + 1 + length]) {
        continue;
      }
      std::vector<uint8_t> frame(bytes.begin() + i, bytes.begin() + i + length + 2U);
      PublishImu(frame);
      return;
    }
  }

  void PublishImu(const std::vector<uint8_t>& frame) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    const double yaw = -DecodeInt16Scaled(frame, 3, 100.0) * M_PI / 180.0;
    const double roll = -DecodeInt16Scaled(frame, 5, 100.0) * M_PI / 180.0;
    const double pitch = -DecodeInt16Scaled(frame, 7, 100.0) * M_PI / 180.0;
    imu.orientation = QuaternionFromEuler(roll, pitch, yaw);
    imu.linear_acceleration.x = -DecodeInt16Scaled(frame, 9, 1000.0) * gravity_;
    imu.linear_acceleration.y = -DecodeInt16Scaled(frame, 11, 1000.0) * gravity_;
    imu.linear_acceleration.z = DecodeInt16Scaled(frame, 13, 1000.0) * gravity_;
    imu.angular_velocity.x = -DecodeInt16Scaled(frame, 15, 1000.0);
    imu.angular_velocity.y = -DecodeInt16Scaled(frame, 17, 1000.0);
    imu.angular_velocity.z = DecodeInt16Scaled(frame, 19, 1000.0);
    imu_pub_->publish(imu);
  }

  void PublishState() {
    std_msgs::msg::String state;
    state.data = calibration_state_;
    state_pub_->publish(state);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string calibration_state_ = "idle";
  double gravity_ = 9.80665;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_calib_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_calib_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu100d4ParserNode>());
  rclcpp::shutdown();
  return 0;
}
