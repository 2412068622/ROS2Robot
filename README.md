# 室内自主移动机器人系统（ROS2 AMR）

一套基于 ROS2 Jazzy + C++17 的室内差速自主移动机器人项目，覆盖从底盘通信、传感器处理、Nav2 导航、SLAM 建图、路径跟踪，到任务调度、站点搬运、设施联动（门 / 电梯 / 充电桩）、车队协同和运营控制的完整软件链路。既能在无真实硬件的情况下通过仿真与 mock 后端完整闭环验证，也提供 Serial / UDP 真实底盘接入和 ros2_control 标准控制链路。

## 功能概览

- **底盘与硬件抽象**：Mock / Serial / UDP 三种底盘后端统一抽象，自定义 `CMD/ODOM/STATE` 行文本协议，并提供 ros2_control `SystemInterface` 插件接入 `diff_drive_controller`。
- **传感器标准化**：激光、IMU 经 filter 标准化为 `/scan`、`/imu/data`，统一坐标系与诊断。
- **导航与定位**：Nav2（basic / advanced）、slam_toolbox、Cartographer、robot_localization EKF 配置齐全，headless 可自动验证 lifecycle。
- **路径跟踪**：Pure Pursuit 与 Stanley 两种控制器，输出带符号横向误差和终点停止判定，可做对比 benchmark。
- **任务调度中枢**：mission_runner 支持任务队列、优先级、抢占、暂停 / 恢复 / 取消、失败恢复策略、成本 / ETA / 电量估算。
- **站点与搬运**：站点目录、站点路网、站点运输订单、坐标运输订单，统一经 `/v2/submit_order` 入口下发。
- **设施联动**：门、电梯、充电桩作为可预约资源，支持预约 / 释放 / 超时回退，门控与电梯会话状态机。
- **充电策略**：低电量自动回桩、任务间隙机会充电、充电桩占用锁。
- **车队与互操作**：多机器人状态管理、车队站点任务分发、VDA 5050 桥接、REST 北向接口。
- **安全控制**：`twist_mux + cmd_vel_safety_gate` 两层（急停 / 限速 / 人工接管），`system_monitor` + `diagnostic_aggregator` + `fault_supervisor` 形成诊断到急停的闭环。
- **仿真与可视化**：Gazebo 室内仓储世界、RViz 标签可视化、一键自动演示编排器（auto demo director）。
- **实验与报告**：场景化 benchmark，输出 CSV / JSON 指标（跟踪误差、吞吐、循环时间等）。

## 技术栈

| 层面 | 技术 |
| --- | --- |
| 语言 | C++17（主体）、Python（launch / 工具 / REST gateway） |
| 框架 | ROS2 Jazzy |
| 构建 | CMake 3.20+ / colcon，GCC 12+ / Clang 15+ |
| 导航 | Nav2（controller / planner / behavior / bt_navigator）、Waypoint Follower |
| 建图定位 | slam_toolbox、Cartographer、robot_localization（EKF） |
| 控制 | ros2_control、diff_drive_controller、Pure Pursuit、Stanley |
| 仿真 | Gazebo（gz）、ros_gz bridge |
| 测试 | GoogleTest（416+ 用例）、launch_testing、shell 验收脚本 |
| 通信协议 | 自定义 Serial/UDP 行协议、VDA 5050（桥接）、REST/HTTP、MQTT（mock 桥接） |

## 系统架构

系统按职责分层，数据自底向上流动：

```text
        Gazebo 仿真 / 真实硬件 / rosbag
                    │
   robot_sensors ── /scan /imu/data /camera/*
                    │
 robot_navigation │ robot_path_tracking │ robot_tasks
   (Nav2 / SLAM)  │ (Pure Pursuit/Stanley)│ (mission 调度)
                    │
   /nav2_cmd_vel /tracking_cmd_vel /teleop_cmd_vel
                    │
   twist_mux ─► /muxed_cmd_vel ─► cmd_vel_safety_gate_node
                    │            (急停 / 限速 / 人工接管)
                  /cmd_vel
                    │
   ros2_control(diff_drive_controller) 或 robot_hardware backend
                    │
   /odom /wheel/odom /odometry/filtered /chassis/state
                    │
   robot_utils: system_monitor ─► /system_health /diagnostics
                fault_supervisor ─► 急停闭环
   robot_navigation: map_manager ─► 地图目录 / 语义区
   robot_tasks: mission_runner ─► /navigate_sequence ─► Nav2 /navigate_to_pose
```



- `/cmd_vel` 默认只能由 `cmd_vel_safety_gate_node` 发布（`use_twist_mux:=false` 时由 `cmd_vel_mux_node` 发布）。
- TF 主链唯一：`map → odom → base_footprint → base_link → {lidar_link, imu_link, camera_link}`。无 EKF 时 driver 发布 `odom→base_footprint`；EKF 开启时由 EKF 独占发布。
- 上层算法只订阅标准传感器 topic，不直接依赖仿真 topic。
- 外部订单统一从 `/v2/submit_order` 进入，`order_type` 路由到 transport / station / fleet / business / scenario / VDA5050 / fulfillment 同一底层链路，不存在第二套提交入口。

## 代码文件架构

```text
robot/
├── src/
│   ├── robot_bringup/        # 顶层启动与配置入口（display / bringup / tracking / amr_demo）
│   ├── robot_description/    # URDF / Xacro 模型、RViz 资产
│   ├── robot_hardware/       # 底盘抽象：协议编解码 + Mock/Serial/UDP 后端 + ros2_control 插件
│   ├── robot_sensors/        # 激光/IMU 标准化、假传感器源、诊断
│   ├── robot_navigation/     # Nav2 / SLAM / EKF 配置、map_manager、语义区 filter mask
│   ├── robot_path_tracking/  # 参考路径发布 + Pure Pursuit / Stanley 控制器
│   ├── robot_tasks/          # 任务中枢：mission 队列/调度/恢复 + 站点/设施/充电/车队/订单
│   ├── robot_teleop/         # twist_mux 安全门、cmd_vel 仲裁、急停、键盘/虚拟遥控
│   ├── robot_simulation/     # Gazebo 世界、仿真桥接、可视化、自动演示编排器
│   ├── robot_utils/          # system_monitor、fault_supervisor 诊断与安全闭环
│   ├── robot_experiments/    # 场景 benchmark runner、参考路径、指标输出
│   └── robot_interfaces*/    # 共享 msg/srv/action，按业务域拆包（core/navigation/mission/
│                             #   facility/fleet/business/site），robot_interfaces 留兼容窗口
├── scripts/                  # 110+ 验收脚本（check_robot.sh 统一入口）、REST gateway、VDA 桥接
├── tools/                    # operator_console.html 单文件运营控制台
├── docker/                   # Dockerfile（ROS2 Jazzy 环境复现）
└── README.md
```

各包代码规模（源文件 / 测试文件）：

| 包 | 职责 | 源 | 测试 |
| --- | --- | --- | --- |
| robot_tasks | 任务调度中枢（含 workflow / behavior tree 拆分） | 46 | 39 |
| robot_hardware | 底盘抽象与协议 | 11 | 2 |
| robot_simulation | Gazebo 仿真桥接与演示 | 6 | — |
| robot_navigation | Nav2 / SLAM / 地图 / 语义区 | 5 | 3 |
| robot_sensors | 传感器标准化 | 4 | — |
| robot_teleop | 安全门与 cmd_vel 仲裁 | 4 | 1 |
| robot_path_tracking | 路径跟踪控制器 | 3 | 1 |
| robot_utils | 诊断与故障监督 | 2 | — |
| robot_experiments | 场景 benchmark | 1 | — |


## 编译

依赖 ROS2 Jazzy（Ubuntu 24.04）。首次构建：

```bash
# 1. 安装系统依赖（rosdep）
cd /home/ubuntu/robot
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 2. 编译
colcon build --symlink-install

# 3. 加载工作空间
source install/setup.bash
```

只编译指定包 / 调试构建：

```bash
colcon build --symlink-install --packages-select robot_tasks robot_navigation
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

用 Docker 复现环境（无需本机装 ROS2）：

```bash
docker build -t robot-amr -f docker/Dockerfile .
docker run -it --rm robot-amr bash      # 进入后已编译好，source install/setup.bash 即可
```

## 在无界面服务器上显示图形界面（VNC）

项目在没有显示器的服务器上运行，Gazebo / RViz 等图形界面通过 **VNC 虚拟桌面 + SSH 端口转发** 投送到本地主机查看（本地用 VNC Viewer，如 `vncviewer64-1.16.2`）。原理是：服务器上 TigerVNC 起一个虚拟桌面并提供 `DISPLAY`，所有 GUI 渲染到该桌面；本机经 SSH 隧道连接 VNC 端口实时显示。

### 1. 服务器端：启动 VNC 虚拟桌面（首次）

```bash
# 首次设置 VNC 密码（仅第一次）
vncpasswd

# 在 display :1 启动虚拟桌面（端口 5901，仅监听 127.0.0.1，需经 SSH 转发访问）
vncserver :1 -geometry 1920x1080 -depth 24 -localhost yes

# 查看 / 复用已有会话
vncserver -list
```

> display 号与端口对应关系：`:1` → `5901`，`:2` → `5902`，依此类推（`5900 + N`）。
> 若桌面空白，确认 `~/.vnc/xstartup` 启动了桌面环境（项目环境已装 `xfce4` / `gnome-session`，例如 `startxfce4 &`）。

### 2. 本地主机：建立 SSH 端口转发

在 **本机** 终端把服务器的 `5901` 隧道到本地 `5901`：

```bash
ssh -L 5901:localhost:5901 ubuntu@<服务器地址>
```

VS Code Remote SSH 用户：也可在端口面板（PORTS）手动 `Forward a Port` 转发 `5901`，效果相同。

### 3. 本地主机：用 VNC Viewer 连接

打开 `vncviewer64-1.16.2`（或任意 VNC Viewer），连接地址填：

```text
localhost:5901
```

输入第 1 步设置的 VNC 密码，即可看到服务器桌面。之后在该桌面里跑带 GUI 的命令（如下方“一键自动仿真”），Gazebo / RViz 窗口就会显示在 VNC 里。

### 4. 让 ROS 图形程序使用该桌面

在 **服务器的 SSH 会话** 里运行 GUI 命令前，把 `DISPLAY` 指向 VNC 桌面：

```bash
export DISPLAY=:1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 之后再 ros2 launch ... gui:=true use_rviz:=true
```

> 在 VNC 桌面内打开的终端通常已自带正确 `DISPLAY`，无需手动 export。
> 完全不需要图形界面时，所有 launch 都支持 headless：`gui:=false use_rviz:=false`。

收尾 / 重启会话：

```bash
vncserver -kill :1        # 关闭桌面
```

## 一键自动仿真（推荐先跑这个）

`amr_demo.launch.py` 是完整的一键自动演示入口：拉起 Gazebo 室内仓储世界、机器人模型、Nav2 序列导航、RViz 可视化，并启动**自动演示编排器**（`amr_sim_demo_director_node`）自动下发一连串任务——巡航、站点搬运、过门 / 电梯、对桩充电、避障停车与恢复，全程无需人工干预。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 带图形界面 + RViz + 自动演示（默认全开）
# 无界面服务器请先按上节起好 VNC，并 export DISPLAY=:1
ros2 launch robot_bringup amr_demo.launch.py gui:=true use_rviz:=true

# 只看自动流程、不弹 RViz
ros2 launch robot_bringup amr_demo.launch.py gui:=true use_rviz:=false

# headless 无图形（CI / 远程服务器）：自动演示仍会跑完整任务链
ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false

# 关闭自动演示，自己手动下发任务
ros2 launch robot_bringup amr_demo.launch.py auto_demo:=false
```

可用参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `use_rviz` | `true` | 是否启动 RViz 可视化 |
| `use_map` | `true` | 是否加载静态地图 |
| `auto_demo` | `true` | 是否启动自动演示编排器自动跑全流程 |
| `labels_enabled` | `false` | 是否启用 Gazebo 内 3D 文字标签（默认关，文字交给 RViz 正对相机显示） |

> 文字标签默认由 RViz 的 `TEXT_VIEW_FACING` marker 显示（永远正对相机、不重叠）；需要纯 Gazebo 演示时可传 `labels_enabled:=true`。

## 运行操作

### 模型显示与基础启动

```bash
# 仅在 RViz 中显示机器人模型与 TF
ros2 launch robot_bringup display.launch.py

# mock 底盘一键起底盘 + 传感器 + 安全链路
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=mock
```

### 真实 / 仿真底盘

```bash
# 串口底盘
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=serial dev:=/dev/ttyUSB0 baud:=115200

# UDP 底盘
ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=udp ip:=192.168.1.10 port:=9000

# 无真实底盘时，本机起一个 UDP 底盘模拟器
ros2 launch robot_hardware chassis_simulator.launch.py bind_host:=127.0.0.1 port:=19000

# ros2_control 标准控制链路（diff_drive_controller）
ros2 launch robot_hardware ros2_control_hardware.launch.py backend:=mock
ros2 control list_controllers

# 接 EKF（robot_localization 独占发布 odom→base_footprint）
ros2 launch robot_hardware hardware_ekf.launch.py backend:=mock
```

### 导航与建图

```bash
# Nav2 导航（可调延迟启动核心导航）
ros2 launch robot_navigation nav.launch.py navigation_start_delay:=5.0

# SLAM 建图
ros2 launch robot_navigation slam.launch.py

# 地图管理（多地图目录扫描、active map 选择、语义区）
ros2 launch robot_navigation map_manager.launch.py

# 语义区 → Nav2 keepout / speed filter mask
ros2 launch robot_navigation zone_filters.launch.py

# 保存与校验地图
scripts/save_map.sh indoor_room src/robot_navigation/maps
scripts/validate_map.sh src/robot_navigation/maps/indoor_room.yaml
```

### 路径跟踪对比实验

```bash
# Pure Pursuit
ros2 launch robot_bringup tracking.launch.py controller:=pure_pursuit
# Stanley
ros2 launch robot_bringup tracking.launch.py controller:=stanley
```

### 任务调度与订单下发

```bash
# 启动任务中枢
ros2 launch robot_tasks mission_runner.launch.py autostart:=true
ros2 launch robot_tasks mission_runner.launch.py return_to_dock_after_mission:=true

# 统一订单入口：坐标运输订单
ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_type: 'transport', payload_json: 'dropoff_x=1.5;dropoff_y=0.8;dropoff_yaw=0'}"

# 站点运输订单
ros2 service call /v2/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_type: 'station_transport', payload_json: 'pickup_station=receiving;dropoff_station=storage_a'}"

# 任务控制
ros2 service call /pause_mission std_srvs/srv/Trigger
ros2 service call /resume_mission std_srvs/srv/Trigger
ros2 service call /return_to_dock std_srvs/srv/Trigger

# 站点 / 设施 / 充电查询
ros2 service call /v2/list_stations robot_interfaces_mission/srv/ListStations
ros2 service call /v2/list_facility_resources robot_interfaces_facility/srv/ListFacilityResources
```

### 安全与诊断

```bash
ros2 service call /enable_emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 service call /clear_emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /emergency_stop/state
ros2 topic echo /system_health
ros2 topic echo /diagnostics_agg
```

### REST 北向接口与运营控制台

```bash
# 启动 REST gateway（默认绑定 127.0.0.1，仅本机；对外暴露需开 --enforce-roles）
python3 scripts/rest_api_gateway.py --host 127.0.0.1 --port 18080

# 提交业务订单
curl -X POST http://127.0.0.1:18080/api/business_orders \
  -H 'Content-Type: application/json' \
  -d '{"business_type":"transport","pickup_station":"receiving","dropoff_station":"storage_a"}'

# 浏览器打开运营控制台（读 /api/operator/snapshot）
xdg-open tools/operator_console.html
```

## 测试与验收

```bash
# 全量构建 + 单元 / 集成测试（GoogleTest + launch_testing）
colcon build --symlink-install
colcon test
colcon test-result --verbose

# 只测某个包
colcon test --packages-select robot_navigation --event-handlers console_direct+

# 统一验收入口（聚合 110+ check 脚本，按 suite/case 调度）
scripts/check_robot.sh                       # 查看可用 suite
scripts/check_robot.sh mission queue         # mission 队列验收
scripts/check_robot.sh facility charging     # 充电 / 设施验收
scripts/check_robot.sh amr_sim assets        # 仿真演示资产验收

# 常用独立验收脚本
scripts/hardware_acceptance.sh mock 10
scripts/udp_sim_acceptance.sh 5
scripts/check_fault_supervisor.sh
scripts/check_mission_runner.sh
```

## 接口

**Topics**：`/scan`、`/imu/data`、`/wheel/odom`、`/odom`、`/odometry/filtered`、`/cmd_vel`（唯一发布者为 safety gate）、`/emergency_stop/state`、`/tracking_error`、`/chassis/state`、`/mission_runner/state`、`/system_health`、`/diagnostics_agg`。

**Action**：`/navigate_sequence`（`robot_interfaces/action/NavigateSequence`，Nav2 可用时调度 `/navigate_to_pose`，否则 mock fallback）。

**Services**：统一订单 `/v2/submit_order`；mission 控制 `/pause_mission`、`/resume_mission`、`/v2/enqueue_mission_profile`、`/v2/preempt_mission_profile`；站点/车队 `/v2/list_stations`、`/v2/submit_fleet_station_task`；设施/充电 `/v2/reserve_facility_resource`、`/v2/request_charging`；安全 `/enable_emergency_stop`、`/v2/set_cmd_source`。接口按业务域拆包 `robot_interfaces_{core,navigation,mission,facility,fleet,business,site}`。

**底盘协议**（Serial/UDP 行文本，每帧 `\n` 结尾）：

```text
ROS → 底盘:  CMD <linear_x_mps> <angular_z_radps>
底盘 → ROS:  ODOM <x_m> <y_m> <yaw_rad> <linear_x_mps> <angular_z_radps> [battery_v]
             STATE <status> <battery_v>
```

运动学当前支持 `diff_drive` 与 `mecanum`；`ackermann` / `four_ws4wd` 等在实现真实转向模型前降级为 `diff_drive`。

## 按岗位分模块学习

本项目覆盖了机器人公司大部分软件岗位的核心技能，可以按目标岗位挑对应模块针对性学习。下表把模块映射到岗位、核心技术点和建议的代码入口。

| 目标岗位 | 主学模块 | 核心技术点 | 代码入口 |
| --- | --- | --- | --- |
| **机器人软件工程师（综合）** | `robot_bringup` + 全栈通读 | ROS2 节点/话题/服务/动作、launch 编排、包间协作 | `src/robot_bringup/launch/`、`src/robot_interfaces*/` |
| **导航 / 运动规划工程师** | `robot_navigation` | Nav2（planner/controller/BT）、costmap、语义区 keepout/speed filter、地图管理 | `src/robot_navigation/`（`map_manager_node`、`zone_filter_masks`、`config/`） |
| **SLAM / 定位工程师** | `robot_navigation` + `robot_sensors` | slam_toolbox、Cartographer、EKF（robot_localization）、TF 树、传感器标定 | `robot_navigation/launch/slam.launch.py`、`robot_hardware/launch/hardware_ekf.launch.py` |
| **运动控制工程师** | `robot_path_tracking` + `robot_hardware` | Pure Pursuit / Stanley、差速/麦轮运动学、ros2_control、`diff_drive_controller` | `robot_path_tracking/src/`、`robot_hardware/src/chassis_kinematics.cpp` |
| **嵌入式 / 底盘驱动工程师** | `robot_hardware` | 串口/UDP 通信、自定义协议编解码、`hardware_interface` 插件、里程计积分 | `robot_hardware/src/`（`chassis_packet`、`serial/udp_backend`、`chassis_hardware_interface`） |
| **感知 / 传感器工程师** | `robot_sensors` | LaserScan/IMU 标准化、滤波、坐标系统一、diagnostics | `src/robot_sensors/src/` |
| **任务调度 / 系统工程师** | `robot_tasks` | 任务队列/优先级/抢占、状态机、behavior tree、失败恢复、成本估算 | `robot_tasks/src/`（`mission_runner_node`、`*_workflow`、`*_behavior_tree`） |
| **AMR 车队 / Fleet 工程师** | `robot_tasks` + `robot_interfaces_*` | 多机调度、站点路网、设施联动（门/电梯/充电）、VDA 5050、订单路由 | `robot_tasks/src/`（`fleet_*`、`facility_*`、`station_*`、`submit_order_router`） |
| **功能安全工程师** | `robot_teleop` + `robot_utils` | cmd_vel 仲裁、急停、watchdog、限速、诊断聚合、故障监督闭环 | `robot_teleop/src/`、`robot_utils/src/`（`system_monitor`、`fault_supervisor`） |
| **仿真 / 工具链工程师** | `robot_simulation` + `robot_description` | Gazebo 世界搭建、ros_gz bridge、URDF/Xacro、RViz 可视化、自动演示编排 | `robot_simulation/`（`worlds/`、`amr_sim_*_node`）、`robot_description/urdf/` |
| **后端 / 上位机 / 集成工程师** | `scripts/` + `tools/` | REST gateway、VDA 5050 桥接、MQTT、webhook 回调、运营控制台 | `scripts/rest_api_gateway.py`、`scripts/vda5050_*`、`tools/operator_console.html` |
| **测试 / DevOps 工程师** | `scripts/` + 各包 `test/` | GoogleTest、launch_testing、headless 验收、CI、Docker | `scripts/check_robot.sh`、各包 `test/`、`.github/`、`docker/` |

### 建议学习路径

- **零基础入门**：先 `robot_bringup` 跑通[一键自动仿真](#一键自动仿真推荐先跑这个) → 读 `robot_description`（机器人长什么样）→ `robot_sensors`（数据从哪来）→ `robot_path_tracking`（最直观的控制闭环）。
- **算法方向**：`robot_navigation`（导航/SLAM）+ `robot_path_tracking`（控制），配合 `robot_experiments` 做对比 benchmark。
- **工程 / 平台方向**：`robot_tasks`（调度中枢，项目最有分量的部分）+ `robot_interfaces_*`（接口设计）+ `scripts/`（北向接口与验收）。
- **底层 / 硬件方向**：`robot_hardware`（协议与 ros2_control）+ `robot_teleop` + `robot_utils`（安全与诊断）。

> 每个模块都能独立编译运行（`colcon build --packages-select <包名>`），可以单点深入而不必通读全项目。

## 许可与说明

本项目为室内 AMR 移动机器人，默认提供 headless 仿真与 mock 后端，可在无真实硬件环境下完整验证；真实硬件接入时须保持 `/cmd_vel`、`/odom`、`/imu/data`、`/chassis/state` 外部契约不变。REST gateway 默认仅绑定本机，对外部署前请开启鉴权、限制请求体大小。

知识星球： “奔跑中cpp / c++” 所有
加入星球即可获得全部资源
星球项目都申请了专利，商业使用前请联系我方授权
一旦发现侵权行为，将依法追究法律责任（对于公司法律事务已有对接律师，敬请告知）
