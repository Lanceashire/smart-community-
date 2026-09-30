# 智慧社区复赛：ROS 2 Jazzy 单圈巡航基线

这是一个**可在已创建的 Jazzy 容器中跑通一圈**的工程起点。它把 RoboMaster 开源项目中适合小场地的速度安全管理和限幅 PID 控制裁剪下来，接到独立制作的 Gazebo Harmonic 场景、巡航状态机和相机灯色识别。当前版本追求先证明 Gazebo → ROS → 控制 → 视觉 → 停车放行 → 完成单圈的闭环；[复赛规则](https://www.aicomp.cn/wp-content/uploads/2026/06/12%E6%99%BA%E6%85%A7%E7%A4%BE%E5%8C%BA.pdf)要求的完整 SLAM、动态灯态、人偶统计、车牌 OCR 和高保真模型还未实现，见[当前边界](#当前边界)。

## 选型与来源

| 上游 | 取用及改造 | 选择理由 |
| --- | --- | --- |
| [Sentry26](https://github.com/Boombroke/Sentry26) | 保留 `sentry_motion_manager` 的指令超时、来源选择、限速、急停入口；将参数收紧至本场地的低速差速底盘，增加 `TwistStamped → Twist` 仿真桥。 | 与现有容器同为 ROS 2 Jazzy + Gazebo Harmonic，原 RM 底盘的 0.46 m 半径和双 Mid360 链路不适合 0.60 m 道路。 |
| [北极熊战队路径控制器](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) | 将其限幅 PID 收缩为 `bounded_pid.py`，分别控制平移与航向；路线节点在大转角先原地转向，接近路点时降速。 | 原件是 ROS 2 Humble 的全向 Nav2 插件，不能直接用于 0.22 m 差速机器人；保留算法核心比整包移植更快。 |

[完整提交来源与版本](NOTICE.md)。还核对了其他哨兵导航方案：中南大学与青岛大学方案主要围绕激光惯性定位和大场地，Aurora-UJS 项目包含 JPS/MINCO 等完整规划链路；这些对首个 4.2 m 单圈演示会引入大量传感器与构建依赖，当前不引入。**没有使用** `Gh0stown/AiC-2026`。

## 本机已验证的环境

- 宿主机：Garuda Linux，Docker 29.8.1。宿主机不需安装 ROS。
- 当前容器：Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.15.0（Harmonic）、`ros_gz_bridge`、`ament_cmake_auto`、`colcon`、OpenCV 4.6。
- 用户现有的开发容器工作目录：`/home/ubuntu/projects/smart-community`，源码放在其中的 `src/`。Gazebo 相机传感器在该容器中实际发布了 640 × 480 RGB 图像；本机的 Nvidia 图形运行时可供它使用。
- 本次基线**不依赖** Ultralytics 权重。YOLO 及车牌 OCR 的训练和接入是后续任务，不能将本版 HSV 灯色检测说成 YOLO 识别。

在新容器中，至少需要以下系统依赖（这里记录所需包，未在新镜像上重新验证从零安装）：

```bash
apt-get update
apt-get install -y python3-colcon-common-extensions python3-opencv \
  ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-ament-cmake-auto ros-jazzy-rclcpp-components \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

需要能运行 Gazebo Ogre2 相机的图形环境。现有容器已具备，不需要重新建容器。

## 编译和运行

进入**已经创建**的容器，然后执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/projects/smart-community
colcon build --symlink-install --packages-select sentry_motion_manager community_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch community_demo demo.launch.py
```

启动后预计约两分钟完成一圈。另开容器终端查看：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /patrol/status
ros2 topic echo /perception/traffic_light
ros2 topic hz /camera/image
ros2 topic hz /scan
```

成功判据是启动终端出现 `REACHED top_stop`、`REACHED lower_stop` 和 `COMPLETE single lap`。`Ctrl+C` 停止 launch。若需要重建静态世界文件，先在仓库根目录运行 `python3 tools/generate_world.py`，然后重新编译 `community_demo`。

## 系统数据流

```text
Gazebo DiffDrive ── odometry ──────────→ community_patrol
Gazebo 360° lidar ── /scan ────────────→ community_patrol（近距离停车）
Gazebo camera ── /camera/image ────────→ community_traffic_vision
                                             │ /perception/traffic_light
                                             ▼
route waypoints ────────────────→ community_patrol
                                    │ /cmd_vel_nav (TwistStamped)
                                    ▼
                         sentry_motion_manager
                                    │ /cmd_vel (TwistStamped)
                                    ▼
                         community_command_bridge
                                    │ /model/patrol_bot/cmd_vel (Twist)
                                    ▼
                               Gazebo DiffDrive
```

Gazebo DiffDrive 的里程计以机器人出生位姿为原点；`patrol.py` 将它变换到世界坐标 `(1.65, 1.55, π)` 后与路线坐标比较。这个变换是调试中确认的，不能直接把里程计的 `(0,0,0)` 当场地原点。

路线按示意图的拓扑顺序经过上侧道路、西侧道路、中间横道、中间纵道、下侧道路和东侧道路返回起点，共 10 个控制路点。上侧与下侧设停止点：先发布零速度，等待连续 5 次相机绿色结果，才推进到下一路点；失去视觉结果时保持停止。`vision.py` 使用 HSV 阈值识别本版模型的亮绿色区域，并做 3 帧一致性检查。近距离 `/scan` 障碍会触发停车，但尚不具备绕行规划。

## 当前边界

1. **已实际验证**：现有容器内两包编译通过；Gazebo 发布相机、雷达和里程计；机器人停靠两处灯位、识别绿灯并完成一个单圈。可在日志中看到 `COMPLETE single lap`。
2. **仿真灯目前固定绿色**，还没有红/黄/绿定时切换，因此不能用这次演示证明“红灯不越线”。
3. 人偶是蓝色/橙色立牌、车牌是蓝色矩形占位模型；尚未贴用户提供的 16＋2 人偶与三张车牌素材，也没有对应的 YOLO 数据集、人数统计或 OCR。
4. 目前用出生位姿已知的里程计和固定路点巡航，没有建图、全局定位、Nav2 路径规划、动态避障或车道线越界检测。它是可运行的工程骨架，**不是可直接提交的完整复赛作品**。
5. Gazebo 世界的尺寸和物料大方向取自随附任务图；本版道具位置仍是近似值，正式仿真应使用材料实测尺寸重新校准。

下一阶段依次加入：可切换的信号灯与停止线验收；用户给的实际贴图与相机取帧；YOLO 人偶/车牌检测和 OCR；SLAM Toolbox/ Nav2；障碍绕行、压线判定、终端和带框图像一一对应输出。每加一个模块就复跑完整巡航。

## 打包

`tools/package.sh` 将源码、世界、README 和来源说明压成 ZIP，排除 `build/`、`install/`、日志和训练数据。复赛官方工程 ZIP 上限为 150 MB；本版代码远低于该上限。训练权重和采集数据以后应单独管理，再按赛事提交要求决定哪些放入工程包。
