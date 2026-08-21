#include <chrono>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/map_zone.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "robot_interfaces_navigation/srv/evaluate_map_position.hpp"
#include "robot_interfaces_navigation/srv/list_map_zones.hpp"
#include "robot_interfaces_navigation/srv/list_maps.hpp"
#include "robot_interfaces_navigation/srv/set_active_map.hpp"
#include "robot_navigation/map_catalog.hpp"
#include "robot_navigation/zone_catalog.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MapManagerNode final : public rclcpp::Node {
 public:
  MapManagerNode() : Node("map_manager_node") {
    const auto default_maps_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_navigation")) /
        "maps";
    const auto default_zones_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_navigation")) /
        "config" / "map_zones.yaml";
    maps_directory_ =
        declare_parameter<std::string>("maps_directory", default_maps_dir.string());
    zones_file_ = declare_parameter<std::string>("zones_file", default_zones_file.string());
    active_map_ = declare_parameter<std::string>("active_map", "indoor_room");
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);
    default_speed_limit_mps_ = declare_parameter<double>("default_speed_limit_mps", 0.6);

    catalog_ = std::make_unique<robot_navigation::MapCatalog>(maps_directory_);
    zone_catalog_ = std::make_unique<robot_navigation::ZoneCatalog>(zones_file_);
    state_pub_ = create_publisher<robot_interfaces::msg::RobotState>("/map_manager/state", 10);
    catalog_pub_ = create_publisher<std_msgs::msg::String>("/map_manager/catalog", 10);
    zones_pub_ = create_publisher<std_msgs::msg::String>("/map_manager/zones", 10);
    v2_list_maps_srv_ = create_service<robot_interfaces_navigation::srv::ListMaps>(
        "/v2/list_maps",
        [this](const std::shared_ptr<robot_interfaces_navigation::srv::ListMaps::Request>,
               std::shared_ptr<robot_interfaces_navigation::srv::ListMaps::Response> response) {
          HandleListMaps(response);
        });
    v2_set_active_map_srv_ = create_service<robot_interfaces_navigation::srv::SetActiveMap>(
        "/v2/set_active_map",
        [this](const std::shared_ptr<robot_interfaces_navigation::srv::SetActiveMap::Request> request,
               std::shared_ptr<robot_interfaces_navigation::srv::SetActiveMap::Response> response) {
          HandleSetActiveMap(request->name, response);
        });
    v2_list_zones_srv_ = create_service<robot_interfaces_navigation::srv::ListMapZones>(
        "/v2/list_map_zones",
        [this](const std::shared_ptr<robot_interfaces_navigation::srv::ListMapZones::Request> request,
               std::shared_ptr<robot_interfaces_navigation::srv::ListMapZones::Response> response) {
          HandleListMapZones(request, response);
        });
    v2_evaluate_position_srv_ =
        create_service<robot_interfaces_navigation::srv::EvaluateMapPosition>(
            "/v2/evaluate_map_position",
            [this](
                const std::shared_ptr<robot_interfaces_navigation::srv::EvaluateMapPosition::Request> request,
                std::shared_ptr<robot_interfaces_navigation::srv::EvaluateMapPosition::Response> response) {
              HandleEvaluateMapPosition(request, response);
            });
    timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(200, publish_period_ms_)),
        [this]() { PublishState(); });
  }

 private:
  std::vector<robot_navigation::MapInfo> Maps() const { return catalog_->Scan(); }

  template <typename ResponseT>
  void HandleListMaps(std::shared_ptr<ResponseT> response) const {
    const auto maps = Maps();
    response->success = !maps.empty();
    response->message = maps.empty() ? "no valid maps found" : "ok";
    response->active_map = active_map_;
    for (const auto& map : maps) {
      response->names.push_back(map.name);
      response->paths.push_back(map.yaml_path.string());
    }
  }

  template <typename ResponseT>
  void HandleSetActiveMap(const std::string& name, std::shared_ptr<ResponseT> response) {
    const auto map = catalog_->FindByName(name);
    if (!map.has_value()) {
      response->success = false;
      response->active_map = active_map_;
      response->message = "map not found or invalid: " + name;
      return;
    }
    active_map_ = map->name;
    response->success = true;
    response->active_map = active_map_;
    response->map_path = map->yaml_path.string();
    response->message = "active map set to " + active_map_;
    PublishState();
  }

  template <typename RequestT, typename ResponseT>
  void HandleListMapZones(
      const std::shared_ptr<RequestT> request, std::shared_ptr<ResponseT> response) const {
    try {
      const auto map_name = request->map_name.empty() ? active_map_ : request->map_name;
      const auto zones = zone_catalog_->ForMap(map_name, request->include_disabled);
      response->success = true;
      response->message = "loaded " + std::to_string(zones.size()) + " zone(s)";
      for (const auto& zone : zones) {
        using ZoneMessageT = typename std::decay_t<decltype(response->zones)>::value_type;
        response->zones.push_back(ToZoneMessage<ZoneMessageT>(zone));
      }
    } catch (const std::exception& error) {
      response->success = false;
      response->message = error.what();
    }
  }

  template <typename RequestT, typename ResponseT>
  void HandleEvaluateMapPosition(
      const std::shared_ptr<RequestT> request, std::shared_ptr<ResponseT> response) const {
    try {
      const auto map_name = request->map_name.empty() ? active_map_ : request->map_name;
      const auto evaluation = zone_catalog_->EvaluatePoint(
          map_name, request->point.x, request->point.y, default_speed_limit_mps_);
      response->success = evaluation.config_valid;
      response->message = evaluation.message;
      response->allowed = evaluation.allowed;
      response->speed_limit_mps = evaluation.speed_limit_mps;
      for (const auto& zone : evaluation.matched_zones) {
        response->matched_zone_ids.push_back(zone.zone_id);
        response->matched_zone_types.push_back(zone.type);
      }
    } catch (const std::exception& error) {
      response->success = false;
      response->message = error.what();
      response->allowed = false;
      response->speed_limit_mps = 0.0;
    }
  }

  void PublishState() {
    const auto maps = Maps();
    const auto active = catalog_->FindByName(active_map_);
    const auto zones_status = BuildZonesText();

    robot_interfaces::msg::RobotState state;
    state.header.stamp = now();
    state.previous_state = previous_state_;
    state.active_goal_id = "";
    state.recoverable = true;
    if (maps.empty()) {
      state.state = "ERROR";
      state.message = "no valid maps found in " + maps_directory_;
      state.recoverable = false;
    } else if (!active.has_value()) {
      state.state = "WARN";
      state.message = "active map is not valid: " + active_map_;
    } else {
      state.state = "OK";
      state.message = "active_map=" + active_map_ + ", maps=" + std::to_string(maps.size());
    }
    previous_state_ = state.state;
    state_pub_->publish(state);

    std_msgs::msg::String catalog_msg;
    catalog_msg.data = BuildCatalogText(maps);
    catalog_pub_->publish(catalog_msg);

    std_msgs::msg::String zones_msg;
    zones_msg.data = zones_status;
    zones_pub_->publish(zones_msg);
  }

  std::string BuildCatalogText(const std::vector<robot_navigation::MapInfo>& maps) const {
    std::ostringstream out;
    out << "active_map=" << active_map_ << "\n";
    for (const auto& map : maps) {
      out << map.name << " " << map.yaml_path.string() << "\n";
    }
    return out.str();
  }

  template <typename ZoneMessageT>
  ZoneMessageT ToZoneMessage(const robot_navigation::MapZone& zone) const {
    ZoneMessageT message;
    message.zone_id = zone.zone_id;
    message.map_name = zone.map_name;
    message.type = zone.type;
    message.frame_id = zone.frame_id;
    message.enabled = zone.enabled;
    message.speed_limit_mps = zone.speed_limit_mps;
    message.polygon_x = zone.polygon_x;
    message.polygon_y = zone.polygon_y;
    message.rule = zone.rule;
    return message;
  }

  std::string BuildZonesText() const {
    std::ostringstream out;
    out << "zones_file=" << zones_file_ << "\n";
    try {
      const auto zones = zone_catalog_->Load(true);
      for (const auto& zone : zones) {
        out << zone.map_name << " " << zone.zone_id << " " << zone.type << " enabled="
            << (zone.enabled ? "true" : "false") << " speed_limit_mps="
            << zone.speed_limit_mps << "\n";
      }
    } catch (const std::exception& error) {
      out << "error=" << error.what() << "\n";
    }
    return out.str();
  }

  std::string maps_directory_;
  std::string zones_file_;
  std::string active_map_;
  std::string previous_state_ = "UNKNOWN";
  int publish_period_ms_ = 1000;
  double default_speed_limit_mps_ = 0.6;
  std::unique_ptr<robot_navigation::MapCatalog> catalog_;
  std::unique_ptr<robot_navigation::ZoneCatalog> zone_catalog_;
  rclcpp::Publisher<robot_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr catalog_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_pub_;
  rclcpp::Service<robot_interfaces_navigation::srv::ListMaps>::SharedPtr v2_list_maps_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::SetActiveMap>::SharedPtr v2_set_active_map_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::ListMapZones>::SharedPtr v2_list_zones_srv_;
  rclcpp::Service<robot_interfaces_navigation::srv::EvaluateMapPosition>::SharedPtr
      v2_evaluate_position_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerNode>());
  rclcpp::shutdown();
  return 0;
}
