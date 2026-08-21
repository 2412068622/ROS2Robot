#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_mission/srv/block_station_route.hpp"
#include "robot_interfaces_mission/srv/cancel_queued_mission.hpp"
#include "robot_interfaces_navigation/srv/clear_obstacle_blockage.hpp"
#include "robot_interfaces_mission/srv/clear_traffic_reservation.hpp"
#include "robot_interfaces_facility/srv/confirm_load.hpp"
#include "robot_interfaces_facility/srv/confirm_unload.hpp"
#include "robot_interfaces_facility/srv/execute_top_module_action.hpp"
#include "robot_interfaces_mission/srv/get_dock_state.hpp"
#include "robot_interfaces_mission/srv/list_mission_events.hpp"
#include "robot_interfaces_navigation/srv/report_obstacle_blockage.hpp"
#include "robot_interfaces_mission/srv/request_docking.hpp"
#include "robot_interfaces_facility/srv/request_lift_session.hpp"
#include "robot_interfaces_mission/srv/request_opportunity_charging.hpp"
#include "robot_interfaces_navigation/srv/request_relocalization.hpp"
#include "robot_interfaces_facility/srv/request_station_confirmation.hpp"
#include "robot_interfaces_mission/srv/request_undocking.hpp"
#include "robot_interfaces_facility/srv/reserve_facility_resource.hpp"
#include "robot_interfaces_navigation/srv/set_dynamic_speed_limit.hpp"
#include "robot_interfaces_navigation/srv/set_initial_pose_from_station.hpp"
#include "robot_interfaces_business/srv/submit_order.hpp"
#include "robot_interfaces_mission/srv/unblock_station_route.hpp"
#include "robot_interfaces_fleet/srv/update_fleet_robot_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AmrSimDemoDirectorNode final : public rclcpp::Node {
 public:
  AmrSimDemoDirectorNode() : Node("amr_sim_demo_director_node") {
    enabled_ = declare_parameter<bool>("enabled", true);
    start_delay_s_ = declare_parameter<double>("start_delay_s", 4.0);
    startup_drive_enabled_ = declare_parameter<bool>("startup_drive_enabled", true);
    startup_odom_topic_ =
        declare_parameter<std::string>("startup_odom_topic", "/model/mobile_robot/odometry");
    startup_cmd_topic_ = declare_parameter<std::string>("startup_cmd_topic", "/virtual_rc/cmd_vel");
    startup_drive_duration_s_ = declare_parameter<double>("startup_drive_duration_s", 8.0);
    startup_drive_linear_mps_ = declare_parameter<double>("startup_drive_linear_mps", 0.45);
    startup_drive_angular_radps_ = declare_parameter<double>("startup_drive_angular_radps", 0.0);
    charging_hold_s_ = declare_parameter<double>("charging_hold_s", 20.0);
    wait_charging_timeout_s_ = declare_parameter<double>("wait_charging_timeout_s", 120.0);
    wait_station_timeout_s_ = declare_parameter<double>("wait_station_timeout_s", 100.0);
    station_arrival_hold_s_ = declare_parameter<double>("station_arrival_hold_s", 4.0);
    traffic_wait_hold_s_ = declare_parameter<double>("traffic_wait_hold_s", 18.0);
    traffic_pass_hold_s_ = declare_parameter<double>("traffic_pass_hold_s", 16.0);
    wait_opportunity_charging_timeout_s_ =
        declare_parameter<double>("wait_opportunity_charging_timeout_s", 180.0);
    obstacle_hold_s_ = declare_parameter<double>("obstacle_hold_s", 90.0);
    obstacle_probe_linear_mps_ = declare_parameter<double>("obstacle_probe_linear_mps", 0.35);
    recovery_drive_duration_s_ = declare_parameter<double>("recovery_drive_duration_s", 4.0);
    tick_period_s_ = declare_parameter<double>("tick_period_s", 0.2);
    timeline_topic_ =
        declare_parameter<std::string>("timeline_topic", "/amr_simulation/demo_timeline");

    load_client_ = create_client<robot_interfaces_facility::srv::ConfirmLoad>("/v2/confirm_load");
    unload_client_ = create_client<robot_interfaces_facility::srv::ConfirmUnload>("/v2/confirm_unload");
    top_module_client_ =
        create_client<robot_interfaces_facility::srv::ExecuteTopModuleAction>("/v2/execute_top_module_action");
    confirmation_client_ = create_client<robot_interfaces_facility::srv::RequestStationConfirmation>(
        "/v2/request_station_confirmation");
    door_client_ = create_client<robot_interfaces_facility::srv::ReserveFacilityResource>(
        "/v2/reserve_facility_resource");
    lift_client_ = create_client<robot_interfaces_facility::srv::RequestLiftSession>("/v2/request_lift_session");
    opportunity_charging_client_ =
        create_client<robot_interfaces_mission::srv::RequestOpportunityCharging>(
            "/v2/request_opportunity_charging");
    relocalization_client_ =
        create_client<robot_interfaces_navigation::srv::RequestRelocalization>("/v2/request_relocalization");
    initial_pose_client_ = create_client<robot_interfaces_navigation::srv::SetInitialPoseFromStation>(
        "/v2/set_initial_pose_from_station");
    block_route_client_ =
        create_client<robot_interfaces_mission::srv::BlockStationRoute>("/v2/block_station_route");
    unblock_route_client_ =
        create_client<robot_interfaces_mission::srv::UnblockStationRoute>("/v2/unblock_station_route");
    speed_client_ =
        create_client<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>("/v2/set_dynamic_speed_limit");
    docking_client_ = create_client<robot_interfaces_mission::srv::RequestDocking>("/v2/request_docking");
    dock_state_client_ = create_client<robot_interfaces_mission::srv::GetDockState>("/v2/get_dock_state");
    undocking_client_ =
        create_client<robot_interfaces_mission::srv::RequestUndocking>("/v2/request_undocking");
    station_order_client_ =
        create_client<robot_interfaces_business::srv::SubmitOrder>("/v2/submit_order");
    clear_traffic_client_ =
        create_client<robot_interfaces_mission::srv::ClearTrafficReservation>("/v2/clear_traffic_reservation");
    cancel_queued_client_ =
        create_client<robot_interfaces_mission::srv::CancelQueuedMission>("/v2/cancel_queued_mission");
    events_client_ =
        create_client<robot_interfaces_mission::srv::ListMissionEvents>("/v2/list_mission_events");
    fleet_state_client_ =
        create_client<robot_interfaces_fleet::srv::UpdateFleetRobotState>("/v2/update_fleet_robot_state");
    obstacle_client_ =
        create_client<robot_interfaces_navigation::srv::ReportObstacleBlockage>("/v2/report_obstacle_blockage");
    clear_obstacle_client_ =
        create_client<robot_interfaces_navigation::srv::ClearObstacleBlockage>("/v2/clear_obstacle_blockage");
    startup_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(startup_cmd_topic_, 10);
    timeline_pub_ =
        create_publisher<std_msgs::msg::String>(timeline_topic_, rclcpp::QoS(1).transient_local());
    startup_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        startup_odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr) {
          startup_odom_received_ = true;
        });

    started_at_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(tick_period_s_)),
        [this]() { Tick(); });
  }

 private:
  enum class Stage {
    kWaitingForServices,
    kStartupDrive,
    kTopModuleInspect,
    kRequestConfirmation,
    kReserveDoor,
    kRequestLift,
    kRequestRelocalization,
    kSetInitialPose,
    kBlockRoute,
    kSetSpeedLimit,
    kRequestDocking,
    kWaitCharging,
    kHoldCharging,
    kRequestUndocking,
    kSubmitStationOrder,
    kWaitStationLoad,
    kConfirmLoad,
    kWaitStationUnload,
    kHoldStationArrival,
    kConfirmUnload,
    kWaitStationFinished,
    kUnblockRoute,
    kSubmitTrafficAlpha,
    kMoveRobot2ToWait,
    kSubmitTrafficBetaBlocked,
    kHoldTrafficWait,
    kReleaseTrafficAlpha,
    kCancelTrafficAlpha,
    kSubmitTrafficBetaAfterRelease,
    kMoveRobot2,
    kHoldTrafficPass,
    kReleaseTrafficBeta,
    kCancelTrafficBeta,
    kRequestOpportunityCharging,
    kWaitOpportunityCharging,
    kReportObstacle,
    kHoldObstacle,
    kClearObstacle,
    kRecoveryDrive,
    kClearSpeedLimit,
    kDone,
  };

  bool DelayElapsed() const {
    return (now() - started_at_).seconds() >= start_delay_s_;
  }

  bool RequiredServicesReady() const {
    return load_client_->service_is_ready() && unload_client_->service_is_ready() &&
           top_module_client_->service_is_ready() &&
           confirmation_client_->service_is_ready() && door_client_->service_is_ready() &&
           lift_client_->service_is_ready() && opportunity_charging_client_->service_is_ready() &&
           relocalization_client_->service_is_ready() && initial_pose_client_->service_is_ready() &&
           block_route_client_->service_is_ready() && unblock_route_client_->service_is_ready() &&
           speed_client_->service_is_ready() && docking_client_->service_is_ready() &&
           dock_state_client_->service_is_ready() && undocking_client_->service_is_ready() &&
           station_order_client_->service_is_ready() && clear_traffic_client_->service_is_ready() &&
           cancel_queued_client_->service_is_ready() && events_client_->service_is_ready() &&
           fleet_state_client_->service_is_ready() && obstacle_client_->service_is_ready() &&
           clear_obstacle_client_->service_is_ready() &&
           (!startup_drive_enabled_ || startup_odom_received_);
  }

  template <typename ServiceT>
  void SendAndAdvance(
      const typename ServiceT::Request::SharedPtr& request,
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const std::string& action_name) {
    if (request_pending_) {
      return;
    }
    request_pending_ = true;
    client->async_send_request(
        request,
        [this, action_name](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(get_logger(), "AMR sim demo action succeeded: %s", action_name.c_str());
          } else {
            RCLCPP_WARN(
                get_logger(), "AMR sim demo action failed: %s: %s", action_name.c_str(),
                response->message.c_str());
          }
          Advance();
        });
  }

  template <typename ServiceT>
  void SendAndAdvanceOnSuccess(
      const typename ServiceT::Request::SharedPtr& request,
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const std::string& action_name) {
    if (request_pending_) {
      return;
    }
    request_pending_ = true;
    client->async_send_request(
        request,
        [this, action_name](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN(
                get_logger(), "AMR sim demo action failed: %s: %s",
                action_name.c_str(), response->message.c_str());
            stage_started_at_ = now();
            return;
          }
          RCLCPP_INFO(
              get_logger(), "AMR sim demo action succeeded: %s",
              action_name.c_str());
          Advance();
        });
  }

  void Advance() {
    stage_ = static_cast<Stage>(static_cast<int>(stage_) + 1);
    stage_started_at_ = now();
  }

  bool StageTimedOut(const double timeout_s) const {
    return (now() - stage_started_at_).seconds() >= timeout_s;
  }

  void Tick() {
    if (!enabled_) {
      return;
    }
    PublishTimeline();
    if (stage_ == Stage::kDone) {
      return;
    }
    if (!DelayElapsed()) {
      return;
    }
    if (stage_ == Stage::kWaitingForServices) {
      if (!RequiredServicesReady()) {
        return;
      }
      RCLCPP_INFO(get_logger(), "AMR sim demo services are ready; starting default scenario");
      Advance();
    }

    switch (stage_) {
      case Stage::kStartupDrive:
        StartupDrive();
        break;
      case Stage::kTopModuleInspect:
        TopModuleInspect();
        break;
      case Stage::kRequestConfirmation:
        RequestConfirmation();
        break;
      case Stage::kReserveDoor:
        ReserveDoor();
        break;
      case Stage::kRequestLift:
        RequestLift();
        break;
      case Stage::kRequestRelocalization:
        RequestRelocalization();
        break;
      case Stage::kSetInitialPose:
        SetInitialPose();
        break;
      case Stage::kBlockRoute:
        BlockRoute();
        break;
      case Stage::kSetSpeedLimit:
        SetSpeedLimit();
        break;
      case Stage::kRequestDocking:
        RequestDocking();
        break;
      case Stage::kWaitCharging:
        WaitCharging();
        break;
      case Stage::kHoldCharging:
        HoldCharging();
        break;
      case Stage::kRequestUndocking:
        RequestUndocking();
        break;
      case Stage::kSubmitStationOrder:
        SubmitStationOrder();
        break;
      case Stage::kWaitStationLoad:
        WaitStationLoad();
        break;
      case Stage::kConfirmLoad:
        ConfirmLoad();
        break;
      case Stage::kWaitStationUnload:
        WaitStationUnload();
        break;
      case Stage::kHoldStationArrival:
        HoldStationArrival();
        break;
      case Stage::kConfirmUnload:
        ConfirmUnload();
        break;
      case Stage::kWaitStationFinished:
        WaitStationFinished();
        break;
      case Stage::kSubmitTrafficAlpha:
        SubmitTrafficAlpha();
        break;
      case Stage::kMoveRobot2ToWait:
        MoveRobot2ToWait();
        break;
      case Stage::kSubmitTrafficBetaBlocked:
        SubmitTrafficBetaBlocked();
        break;
      case Stage::kHoldTrafficWait:
        HoldTrafficWait();
        break;
      case Stage::kReleaseTrafficAlpha:
        ReleaseTrafficAlpha();
        break;
      case Stage::kCancelTrafficAlpha:
        CancelTrafficAlpha();
        break;
      case Stage::kSubmitTrafficBetaAfterRelease:
        SubmitTrafficBetaAfterRelease();
        break;
      case Stage::kMoveRobot2:
        MoveRobot2();
        break;
      case Stage::kHoldTrafficPass:
        HoldTrafficPass();
        break;
      case Stage::kReleaseTrafficBeta:
        ReleaseTrafficBeta();
        break;
      case Stage::kCancelTrafficBeta:
        CancelTrafficBeta();
        break;
      case Stage::kRequestOpportunityCharging:
        RequestOpportunityCharging();
        break;
      case Stage::kWaitOpportunityCharging:
        WaitOpportunityCharging();
        break;
      case Stage::kReportObstacle:
        ReportObstacle();
        break;
      case Stage::kHoldObstacle:
        HoldObstacle();
        break;
      case Stage::kClearObstacle:
        ClearObstacle();
        break;
      case Stage::kRecoveryDrive:
        RecoveryDrive();
        break;
      case Stage::kUnblockRoute:
        UnblockRoute();
        break;
      case Stage::kClearSpeedLimit:
        ClearSpeedLimit();
        break;
      case Stage::kWaitingForServices:
      case Stage::kDone:
        break;
    }
  }

  static std::string StageName(const Stage stage) {
    switch (stage) {
      case Stage::kWaitingForServices:
        return "等待服务就绪";
      case Stage::kStartupDrive:
        return "启动前进";
      case Stage::kTopModuleInspect:
        return "上装检查";
      case Stage::kRequestConfirmation:
        return "等待人工确认";
      case Stage::kReserveDoor:
        return "门禁通行";
      case Stage::kRequestLift:
        return "电梯会话";
      case Stage::kRequestRelocalization:
        return "定位恢复请求";
      case Stage::kSetInitialPose:
        return "定位恢复";
      case Stage::kBlockRoute:
        return "交通封路";
      case Stage::kSetSpeedLimit:
        return "动态限速";
      case Stage::kRequestDocking:
        return "对桩重试";
      case Stage::kWaitCharging:
        return "自动充电";
      case Stage::kHoldCharging:
        return "充电保持";
      case Stage::kRequestUndocking:
        return "脱离充电桩";
      case Stage::kSubmitStationOrder:
        return "收货口到货架B运输";
      case Stage::kWaitStationLoad:
        return "前往装货点";
      case Stage::kConfirmLoad:
        return "装货确认";
      case Stage::kWaitStationUnload:
        return "前往卸货点";
      case Stage::kHoldStationArrival:
        return "到站停靠";
      case Stage::kConfirmUnload:
        return "卸货确认";
      case Stage::kWaitStationFinished:
        return "运输收尾";
      case Stage::kSubmitTrafficAlpha:
        return "一号车占用路口";
      case Stage::kMoveRobot2ToWait:
        return "二号车到等待点";
      case Stage::kSubmitTrafficBetaBlocked:
        return "二号车请求被等待";
      case Stage::kHoldTrafficWait:
        return "两车会车等待";
      case Stage::kReleaseTrafficAlpha:
        return "释放交通资源";
      case Stage::kCancelTrafficAlpha:
        return "清理等待任务";
      case Stage::kSubmitTrafficBetaAfterRelease:
        return "二号车获得通行权";
      case Stage::kMoveRobot2:
        return "第二台车通行";
      case Stage::kHoldTrafficPass:
        return "第二台车通过路口";
      case Stage::kReleaseTrafficBeta:
        return "二号车释放路口";
      case Stage::kCancelTrafficBeta:
        return "清理通行任务";
      case Stage::kRequestOpportunityCharging:
        return "机会充电";
      case Stage::kWaitOpportunityCharging:
        return "机会充电执行中";
      case Stage::kReportObstacle:
        return "障碍阻塞";
      case Stage::kHoldObstacle:
        return "安全停车等待";
      case Stage::kClearObstacle:
        return "清障恢复";
      case Stage::kRecoveryDrive:
        return "清障后恢复行驶";
      case Stage::kUnblockRoute:
        return "路段恢复";
      case Stage::kClearSpeedLimit:
        return "限速解除";
      case Stage::kDone:
        return "演示完成";
    }
    return "未知阶段";
  }

  static Stage NextStage(const Stage stage) {
    if (stage == Stage::kDone) {
      return Stage::kDone;
    }
    return static_cast<Stage>(static_cast<int>(stage) + 1);
  }

  void PublishTimeline() const {
    std_msgs::msg::String msg;
    const int stage_index = static_cast<int>(stage_);
    const int final_stage_index = static_cast<int>(Stage::kDone);
    msg.data = "当前阶段: " + StageName(stage_) + "\n下一阶段: " + StageName(NextStage(stage_)) +
               "\n进度: " + std::to_string(stage_index) + "/" +
               std::to_string(final_stage_index);
    timeline_pub_->publish(msg);
  }

  void StartupDrive() {
    if (!startup_drive_enabled_) {
      Advance();
      return;
    }
    if (StageTimedOut(startup_drive_duration_s_)) {
      startup_cmd_pub_->publish(geometry_msgs::msg::Twist{});
      Advance();
      return;
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = startup_drive_linear_mps_;
    twist.angular.z = startup_drive_angular_radps_;
    startup_cmd_pub_->publish(twist);
  }

  void TopModuleInspect() {
    auto request = std::make_shared<robot_interfaces_facility::srv::ExecuteTopModuleAction::Request>();
    request->action = "inspect";
    request->payload_id = "sim_auto_tote";
    request->weight_kg = 8.5;
    SendAndAdvance<robot_interfaces_facility::srv::ExecuteTopModuleAction>(
        request, top_module_client_, "top module inspect");
  }

  void RequestConfirmation() {
    auto request = std::make_shared<robot_interfaces_facility::srv::RequestStationConfirmation::Request>();
    request->confirmation_id = "sim_auto_load_confirm";
    request->station_id = "receiving";
    request->action = "load_tote";
    request->mission_id = "sim_auto_demo";
    request->operator_hint = "default AMR simulation loading confirmation";
    request->timeout_sec = 0;
    request->timeout_policy = "manual";
    SendAndAdvance<robot_interfaces_facility::srv::RequestStationConfirmation>(
        request, confirmation_client_, "request station confirmation");
  }

  void ReserveDoor() {
    auto request = std::make_shared<robot_interfaces_facility::srv::ReserveFacilityResource::Request>();
    request->resource_id = "receiving_door";
    request->holder_id = "sim_auto_door";
    request->mission_id = "sim_auto_demo";
    request->release_existing_for_holder = true;
    SendAndAdvance<robot_interfaces_facility::srv::ReserveFacilityResource>(
        request, door_client_, "reserve receiving door");
  }

  void RequestLift() {
    auto request = std::make_shared<robot_interfaces_facility::srv::RequestLiftSession::Request>();
    request->session_id = "sim_auto_lift";
    request->lift_id = "freight_elevator";
    request->requester_id = "sim_auto_demo";
    request->from_station = "packing";
    request->to_station = "floor_2_dropoff";
    request->release_existing_for_requester = true;
    SendAndAdvance<robot_interfaces_facility::srv::RequestLiftSession>(
        request, lift_client_, "request lift session");
  }

  void RequestRelocalization() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::RequestRelocalization::Request>();
    request->reason = "default AMR simulation relocalization drill";
    SendAndAdvance<robot_interfaces_navigation::srv::RequestRelocalization>(
        request, relocalization_client_, "request relocalization");
  }

  void SetInitialPose() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::SetInitialPoseFromStation::Request>();
    request->station_id = "dock";
    SendAndAdvance<robot_interfaces_navigation::srv::SetInitialPoseFromStation>(
        request, initial_pose_client_, "set initial pose from dock");
  }

  void BlockRoute() {
    auto request = std::make_shared<robot_interfaces_mission::srv::BlockStationRoute::Request>();
    request->from_station = "receiving";
    request->to_station = "storage_a";
    request->reason = "default AMR simulation blocked aisle";
    SendAndAdvance<robot_interfaces_mission::srv::BlockStationRoute>(
        request, block_route_client_, "block station route");
  }

  void SetSpeedLimit() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Request>();
    request->speed_limit_mps = 0.45;
    request->reason = "default AMR simulation slow zone";
    SendAndAdvance<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>(
        request, speed_client_, "set dynamic speed limit");
  }

  void RequestDocking() {
    auto request = std::make_shared<robot_interfaces_mission::srv::RequestDocking::Request>();
    request->dock_id = "main_dock";
    request->request_id = "sim_auto";
    request->priority = 50;
    request->start_if_idle = true;
    request->preempt_current = false;
    request->simulate_contact_success = false;
    SendAndAdvance<robot_interfaces_mission::srv::RequestDocking>(
        request, docking_client_, "request docking");
  }

  void WaitCharging() {
    if (request_pending_) {
      return;
    }
    if (StageTimedOut(wait_charging_timeout_s_)) {
      RCLCPP_WARN(get_logger(), "AMR sim demo docking wait timed out; continuing scenario");
      Advance();
      return;
    }
    request_pending_ = true;
    auto request = std::make_shared<robot_interfaces_mission::srv::GetDockState::Request>();
    request->dock_id = "main_dock";
    dock_state_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_mission::srv::GetDockState>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (response->success && response->state.find("CHARG") != std::string::npos) {
            RCLCPP_INFO(get_logger(), "AMR sim demo dock reached state: %s", response->state.c_str());
            Advance();
          }
        });
  }

  void HoldCharging() {
    if (StageTimedOut(charging_hold_s_)) {
      Advance();
    }
  }

  void RequestUndocking() {
    auto request = std::make_shared<robot_interfaces_mission::srv::RequestUndocking::Request>();
    request->dock_id = "main_dock";
    request->request_id = "sim_auto";
    request->release_charger = true;
    SendAndAdvance<robot_interfaces_mission::srv::RequestUndocking>(
        request, undocking_client_, "request undocking");
  }

  void SubmitStationOrder() {
    SubmitStationTransportOrder(
        "sim_auto_transport", "receiving", "storage_b", 30, true, false,
        "submit station transport");
  }

  void WaitStationLoad() {
    WaitMissionState(
        "station_order_sim_auto_transport", "WAIT_LOAD", wait_station_timeout_s_,
        "station transport pickup arrival");
  }

  void ConfirmLoad() {
    if ((now() - stage_started_at_).seconds() < 1.0) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::ConfirmLoad::Request>();
    request->payload_id = "sim_auto_tote";
    request->weight_kg = 8.5;
    request->station_id = "receiving";
    SendAndAdvanceOnSuccess<robot_interfaces_facility::srv::ConfirmLoad>(
        request, load_client_, "confirm payload load at receiving");
  }

  void WaitStationUnload() {
    WaitMissionState(
        "station_order_sim_auto_transport", "WAIT_UNLOAD", wait_station_timeout_s_,
        "station transport dropoff arrival");
  }

  void WaitStationFinished() {
    WaitMissionState(
        "station_order_sim_auto_transport", "FINISHED", wait_station_timeout_s_,
        "station transport completion");
  }

  void WaitMissionState(
      const std::string& mission_id, const std::string& expected_state,
      const double timeout_s, const std::string& label) {
    if (request_pending_) {
      return;
    }
    if (StageTimedOut(timeout_s)) {
      RCLCPP_WARN(
          get_logger(), "AMR sim demo %s is still waiting for %s",
          label.c_str(), expected_state.c_str());
      stage_started_at_ = now();
      return;
    }
    request_pending_ = true;
    auto request = std::make_shared<robot_interfaces_mission::srv::ListMissionEvents::Request>();
    request->limit = 10;
    request->mission_id_filter = mission_id;
    request->state_filter = expected_state;
    events_client_->async_send_request(
        request,
        [this, label, expected_state](
            rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (response->success && !response->mission_ids.empty()) {
            RCLCPP_INFO(
                get_logger(), "AMR sim demo %s reached %s", label.c_str(),
                expected_state.c_str());
            Advance();
          }
        });
  }

  void ReportObstacle() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::ReportObstacleBlockage::Request>();
    request->blockage_id = "sim_auto_obstacle";
    request->reason = "default AMR simulation obstacle";
    request->pause_active = false;
    SendAndAdvance<robot_interfaces_navigation::srv::ReportObstacleBlockage>(
        request, obstacle_client_, "report simulated obstacle");
  }

  void HoldObstacle() {
    PublishObstacleProbeCommand();
    if (StageTimedOut(obstacle_hold_s_)) {
      startup_cmd_pub_->publish(geometry_msgs::msg::Twist{});
      Advance();
    }
  }

  void ClearObstacle() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::ClearObstacleBlockage::Request>();
    request->blockage_id = "sim_auto_obstacle";
    request->resolution = "default AMR simulation obstacle cleared";
    request->resume_active = false;
    SendAndAdvance<robot_interfaces_navigation::srv::ClearObstacleBlockage>(
        request, clear_obstacle_client_, "clear simulated obstacle");
  }

  void PublishObstacleProbeCommand() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = obstacle_probe_linear_mps_;
    startup_cmd_pub_->publish(twist);
  }

  void RecoveryDrive() {
    if (StageTimedOut(recovery_drive_duration_s_)) {
      startup_cmd_pub_->publish(geometry_msgs::msg::Twist{});
      Advance();
      return;
    }
    PublishObstacleProbeCommand();
  }

  void UnblockRoute() {
    auto request = std::make_shared<robot_interfaces_mission::srv::UnblockStationRoute::Request>();
    request->from_station = "receiving";
    request->to_station = "storage_a";
    SendAndAdvance<robot_interfaces_mission::srv::UnblockStationRoute>(
        request, unblock_route_client_, "unblock station route");
  }

  void ClearSpeedLimit() {
    auto request = std::make_shared<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Request>();
    request->speed_limit_mps = 0.0;
    request->reason = "default AMR simulation slow zone cleared";
    SendAndAdvance<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>(
        request, speed_client_, "clear dynamic speed limit");
  }

  void HoldStationArrival() {
    if (StageTimedOut(station_arrival_hold_s_)) {
      Advance();
    }
  }

  void ConfirmUnload() {
    if ((now() - stage_started_at_).seconds() < 1.0) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_facility::srv::ConfirmUnload::Request>();
    request->payload_id = "sim_auto_tote";
    request->station_id = "storage_b";
    SendAndAdvanceOnSuccess<robot_interfaces_facility::srv::ConfirmUnload>(
        request, unload_client_, "confirm payload unload at storage_b");
  }

  void SubmitTrafficAlpha() {
    SubmitTrafficOrder(
        "traffic_alpha", "receiving", "storage_a", 28, false,
        "submit alpha traffic reservation");
  }

  void MoveRobot2ToWait() {
    UpdateRobot2FleetState("WAITING_TRAFFIC", "traffic_wait", "move robot_2 to traffic wait point");
  }

  void SubmitTrafficBetaBlocked() {
    SubmitTrafficOrder(
        "traffic_beta", "storage_a", "packing", 27, false,
        "submit blocked beta traffic request");
  }

  void HoldTrafficWait() {
    if (StageTimedOut(traffic_wait_hold_s_)) {
      Advance();
    }
  }

  void ReleaseTrafficAlpha() {
    ClearTrafficReservation("route_edge:receiving__storage_a", "release alpha traffic reservation");
  }

  void CancelTrafficAlpha() {
    CancelQueuedMission("station_order_traffic_alpha", "cancel alpha traffic queue entry");
  }

  void SubmitTrafficBetaAfterRelease() {
    SubmitTrafficOrder(
        "traffic_beta_after_release", "storage_a", "packing", 29, false,
        "submit beta traffic request after release");
  }

  void HoldTrafficPass() {
    if (StageTimedOut(traffic_pass_hold_s_)) {
      Advance();
    }
  }

  void ReleaseTrafficBeta() {
    ClearTrafficReservation("route_edge:packing__storage_a", "release beta traffic reservation");
  }

  void CancelTrafficBeta() {
    CancelQueuedMission(
        "station_order_traffic_beta_after_release", "cancel beta traffic queue entry");
  }

  void SubmitTrafficOrder(
      const std::string& order_id, const std::string& pickup_station,
      const std::string& dropoff_station, const int priority, const bool start_if_idle,
      const std::string& action_name) {
    SubmitStationTransportOrder(
        order_id, pickup_station, dropoff_station, priority, start_if_idle, false,
        action_name);
  }

  void SubmitStationTransportOrder(
      const std::string& order_id, const std::string& pickup_station,
      const std::string& dropoff_station, const int priority, const bool start_if_idle,
      const bool preempt_current, const std::string& action_name) {
    if (request_pending_) {
      return;
    }
    auto request = std::make_shared<robot_interfaces_business::srv::SubmitOrder::Request>();
    request->order_id = order_id;
    request->order_type = "station_transport";
    request->priority = priority;
    request->payload_json = "pickup_station=" + pickup_station +
                            ";dropoff_station=" + dropoff_station +
                            ";start_if_idle=" + (start_if_idle ? std::string{"true"} : "false") +
                            ";preempt_current=" +
                            (preempt_current ? std::string{"true"} : "false");
    request->tags = {"amr_sim_demo_director"};
    request_pending_ = true;
    station_order_client_->async_send_request(
        request,
        [this, action_name](rclcpp::Client<robot_interfaces_business::srv::SubmitOrder>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (response->accepted) {
            RCLCPP_INFO(get_logger(), "AMR sim demo action succeeded: %s", action_name.c_str());
          } else {
            RCLCPP_WARN(
                get_logger(), "AMR sim demo action failed: %s: %s", action_name.c_str(),
                response->message.c_str());
          }
          Advance();
        });
  }

  void ClearTrafficReservation(const std::string& resource_id, const std::string& action_name) {
    auto request = std::make_shared<robot_interfaces_mission::srv::ClearTrafficReservation::Request>();
    request->resource_id = resource_id;
    SendAndAdvance<robot_interfaces_mission::srv::ClearTrafficReservation>(
        request, clear_traffic_client_, action_name);
  }

  void CancelQueuedMission(const std::string& mission_id, const std::string& action_name) {
    auto request = std::make_shared<robot_interfaces_mission::srv::CancelQueuedMission::Request>();
    request->mission_id = mission_id;
    request->cancel_active = false;
    SendAndAdvance<robot_interfaces_mission::srv::CancelQueuedMission>(
        request, cancel_queued_client_, action_name);
  }

  void RequestOpportunityCharging() {
    auto request = std::make_shared<robot_interfaces_mission::srv::RequestOpportunityCharging::Request>();
    request->request_id = "sim_auto_opportunity";
    request->charger_id = "charger_main";
    request->battery_voltage = 22.8;
    request->opportunity_threshold_voltage = 23.2;
    request->critical_threshold_voltage = 22.0;
    request->priority = 45;
    request->start_if_idle = true;
    request->preempt_current = false;
    request->require_idle_queue = true;
    SendAndAdvance<robot_interfaces_mission::srv::RequestOpportunityCharging>(
        request, opportunity_charging_client_, "request opportunity charging");
  }

  void WaitOpportunityCharging() {
    if (request_pending_) {
      return;
    }
    if (StageTimedOut(wait_opportunity_charging_timeout_s_)) {
      RCLCPP_WARN(get_logger(), "AMR sim demo opportunity charging wait timed out");
      Advance();
      return;
    }
    request_pending_ = true;
    auto request = std::make_shared<robot_interfaces_mission::srv::ListMissionEvents::Request>();
    request->limit = 10;
    request->mission_id_filter = "charging_request_sim_auto_opportunity";
    request->state_filter = "FINISHED";
    events_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedFuture future) {
          request_pending_ = false;
          const auto response = future.get();
          if (response->success && !response->mission_ids.empty()) {
            RCLCPP_INFO(get_logger(), "AMR sim demo opportunity charging finished");
            Advance();
          }
        });
  }

  void MoveRobot2() {
    UpdateRobot2FleetState("MOVING", "packing", "release robot_2 to packing");
  }

  void UpdateRobot2FleetState(
      const std::string& state, const std::string& station_id, const std::string& action_name) {
    auto request = std::make_shared<robot_interfaces_fleet::srv::UpdateFleetRobotState::Request>();
    request->robot_id = "robot_2";
    request->state = state;
    request->battery_voltage = 24.0;
    request->current_station_id = station_id;
    request->update_state = true;
    request->update_current_station_id = true;
    SendAndAdvance<robot_interfaces_fleet::srv::UpdateFleetRobotState>(
        request, fleet_state_client_, action_name);
  }

  bool enabled_ = true;
  double start_delay_s_ = 4.0;
  bool startup_drive_enabled_ = true;
  std::string startup_odom_topic_;
  std::string startup_cmd_topic_;
  std::string timeline_topic_;
  bool startup_odom_received_ = false;
  double startup_drive_duration_s_ = 8.0;
  double startup_drive_linear_mps_ = 0.45;
  double startup_drive_angular_radps_ = 0.0;
  double charging_hold_s_ = 20.0;
  double wait_charging_timeout_s_ = 120.0;
  double wait_station_timeout_s_ = 100.0;
  double station_arrival_hold_s_ = 4.0;
  double traffic_wait_hold_s_ = 18.0;
  double traffic_pass_hold_s_ = 16.0;
  double wait_opportunity_charging_timeout_s_ = 180.0;
  double obstacle_hold_s_ = 90.0;
  double obstacle_probe_linear_mps_ = 0.35;
  double recovery_drive_duration_s_ = 4.0;
  double tick_period_s_ = 0.2;
  bool request_pending_ = false;
  Stage stage_ = Stage::kWaitingForServices;
  rclcpp::Time started_at_;
  rclcpp::Time stage_started_at_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<robot_interfaces_facility::srv::ConfirmLoad>::SharedPtr load_client_;
  rclcpp::Client<robot_interfaces_facility::srv::ConfirmUnload>::SharedPtr unload_client_;
  rclcpp::Client<robot_interfaces_facility::srv::ExecuteTopModuleAction>::SharedPtr top_module_client_;
  rclcpp::Client<robot_interfaces_facility::srv::RequestStationConfirmation>::SharedPtr confirmation_client_;
  rclcpp::Client<robot_interfaces_facility::srv::ReserveFacilityResource>::SharedPtr door_client_;
  rclcpp::Client<robot_interfaces_facility::srv::RequestLiftSession>::SharedPtr lift_client_;
  rclcpp::Client<robot_interfaces_mission::srv::RequestOpportunityCharging>::SharedPtr
      opportunity_charging_client_;
  rclcpp::Client<robot_interfaces_navigation::srv::RequestRelocalization>::SharedPtr relocalization_client_;
  rclcpp::Client<robot_interfaces_navigation::srv::SetInitialPoseFromStation>::SharedPtr initial_pose_client_;
  rclcpp::Client<robot_interfaces_mission::srv::BlockStationRoute>::SharedPtr block_route_client_;
  rclcpp::Client<robot_interfaces_mission::srv::UnblockStationRoute>::SharedPtr unblock_route_client_;
  rclcpp::Client<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>::SharedPtr speed_client_;
  rclcpp::Client<robot_interfaces_mission::srv::RequestDocking>::SharedPtr docking_client_;
  rclcpp::Client<robot_interfaces_mission::srv::GetDockState>::SharedPtr dock_state_client_;
  rclcpp::Client<robot_interfaces_mission::srv::RequestUndocking>::SharedPtr undocking_client_;
  rclcpp::Client<robot_interfaces_business::srv::SubmitOrder>::SharedPtr station_order_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ClearTrafficReservation>::SharedPtr clear_traffic_client_;
  rclcpp::Client<robot_interfaces_mission::srv::CancelQueuedMission>::SharedPtr cancel_queued_client_;
  rclcpp::Client<robot_interfaces_mission::srv::ListMissionEvents>::SharedPtr events_client_;
  rclcpp::Client<robot_interfaces_fleet::srv::UpdateFleetRobotState>::SharedPtr fleet_state_client_;
  rclcpp::Client<robot_interfaces_navigation::srv::ReportObstacleBlockage>::SharedPtr obstacle_client_;
  rclcpp::Client<robot_interfaces_navigation::srv::ClearObstacleBlockage>::SharedPtr clear_obstacle_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr startup_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timeline_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr startup_odom_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmrSimDemoDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
