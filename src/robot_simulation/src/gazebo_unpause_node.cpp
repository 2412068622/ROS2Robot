#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"

class GazeboUnpauseNode final : public rclcpp::Node {
 public:
  GazeboUnpauseNode() : Node("gazebo_unpause_node") {
    service_name_ =
        declare_parameter<std::string>("service_name", "/world/indoor_room/control");
    max_attempts_ = declare_parameter<int>("max_attempts", 30);
    retry_period_s_ = declare_parameter<double>("retry_period_s", 0.5);

    if (max_attempts_ < 1) {
      max_attempts_ = 1;
    }
    if (retry_period_s_ <= 0.0) {
      retry_period_s_ = 0.5;
    }

    client_ = create_client<ros_gz_interfaces::srv::ControlWorld>(service_name_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(retry_period_s_)),
        [this]() { TryUnpause(); });
  }

 private:
  void TryUnpause() {
    if (request_in_flight_) {
      return;
    }

    ++attempts_;
    if (!client_->service_is_ready()) {
      if (attempts_ >= max_attempts_) {
        RCLCPP_WARN(
            get_logger(), "Gazebo control service [%s] was not available after %d attempts",
            service_name_.c_str(), attempts_);
        rclcpp::shutdown();
      }
      return;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    request->world_control.pause = false;
    request_in_flight_ = true;
    client_->async_send_request(
        request,
        [this](rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedFuture future) {
          request_in_flight_ = false;
          const auto response = future.get();
          if (response != nullptr && response->success) {
            RCLCPP_INFO(get_logger(), "Gazebo world unpaused through [%s]", service_name_.c_str());
            rclcpp::shutdown();
            return;
          }

          if (attempts_ >= max_attempts_) {
            RCLCPP_WARN(
                get_logger(), "Gazebo world unpause did not succeed after %d attempts", attempts_);
            rclcpp::shutdown();
          }
        });
  }

  std::string service_name_;
  int max_attempts_{30};
  int attempts_{0};
  double retry_period_s_{0.5};
  bool request_in_flight_{false};
  rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboUnpauseNode>());
  rclcpp::shutdown();
  return 0;
}
