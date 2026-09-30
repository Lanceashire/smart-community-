# 智慧社区复赛：ROS 2 Jazzy 单圈巡航基线

这是一个**可在已创建的 Jazzy 容器中跑通一圈**的工程起点。它把 RoboMaster 开源项目中适合小场地的速度安全管理和限幅 PID 控制裁剪下来，接到独立制作的 Gazebo Harmonic 场景、巡航状态机和相机灯色识别。当前版本追求先证明 Gazebo → ROS → 控制 → 视觉 → 停车放行 → 完成单圈的闭环；[复赛规则](https://www.aicomp.cn/wp-content/uploads/2026/06/12%E6%99%BA%E6%85%A7%E7%A4%BE%E5%8C%BA.pdf)要求的完整 SLAM、动态灯态、人偶统计、车牌 OCR 和高保真模型还未实现，见[当前边界](#当前边界)。

## 选型与来源

| 上游 | 取用及改造 | 选择理由 |
| --- | --- | --- |
| [Sentry26](https://github.com/Boombroke/Sentry26) | 保留 `sentry_motion_manager` 的指令超时、来源选择、限速、急停入口；将参数收紧至本场地的低速差速底盘，增加 `TwistStamped → Twist` 仿真桥。 | 与现有容器同为 ROS 2 Jazzy + Gazebo Harmonic，原 RM 底盘的 0.46 m 半径和双 Mid360 链路不适合 0.60 m 道路。 |
| [北极熊战队路径控制器](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) | 将其限幅 PID 收缩为 `bounded_pid.py`，分别控制平移与航向；路线节点在大转角先原地转向，接近路点时降速。 | 原件是 ROS 2 Humble 的全向 Nav2 插件，不能直接用于 0.22 m 差速机器人；保留算法核心比整包移植更快。 |

[完整提交来源与版本](NOTICE.md)。还核对了其他哨兵导航方案：中南大学与青岛大学方案主要围绕激光惯性定位和大场地，Aurora-UJS 项目包含 JPS/MINCO 等完整规划链路；这些对首个 4.2 m 单圈演示会引入大量传感器与构建依赖，当前不引入。**没有使用** `Gh0stown/AiC-2026`。

## 2026-09-30 物理运动修复

最初版本只查看了 DiffDrive 轮里程计和控制日志，错误地把轮子转动判断为完成巡航。桌面复核发现两轮底盘缺少支撑而倾倒，真实车体只偏移约 2.5 mm；旧提交的“跑通”声明不成立。本版据此作了以下修复：

- 增加前后低摩擦球形支撑，两个驱动轮的旋转轴显式指定为模型坐标系的 +Y；底盘不再只靠一根轮轴保持平衡。
- 增加粉色顶盖、黄色车头、雷达和相机外观，打开俯视窗口即可识别机器人。
- 使用 Gazebo `OdometryPublisher` 的**物理世界位姿**驱动这个仿真基线，轮里程计保留作诊断。真实车体移动不足时不能完成路线；倾角超过 20°、位姿失联、连续 4 秒有指令但无实际运动都会停车。
- 右侧巡航线移动到 `x=1.35 m`，与停放车辆保持间距；相机俯仰角按水平底盘重新校准。
- GUI 与仿真服务器由同一条 launch 命令启动；增加 `/patrol/restart`，完成后可原地再次出发，无需关闭窗口。
- 雷达扫描高度为 0.235 m，低于 0.255 m 车顶与 0.24 m 场地墙；接收数据超过 0.7 秒未更新时停车。停车放行要求停靠后收到 5 条新的绿色视觉结果，避免反复使用同一条结果计数。

最终物理验收通过：83.55 秒、真实轨迹 11.53 m、X/Y 跨度 2.90/3.09 m、全部 10 个路点到达，最大倾角 0.0016° 以下；三辆停放车的雷达测距均与车体几何位置吻合。见[验收摘要](validation/physical-lap-2026-09-30.json)，复现工具为 `tools/verify_physical_lap.py`。

这里的物理世界位姿是**仿真专用定位输入**，不是 SLAM 或真实机器人可用的定位结果。验收脚本同时要求真实轨迹长度、X/Y 跨度、倾角和全部路点通过，不能只检查日志文字。

## 本机已验证的环境

- 宿主机：Garuda Linux，Docker 29.8.1。宿主机不需安装 ROS。
- 当前容器：Ubuntu 24.04.4、ROS 2 Jazzy、Gazebo Sim 8.15.0（Harmonic）、`ros_gz_bridge`、`ament_cmake_auto`、`colcon`、OpenCV 4.6。
- 用户现有的开发容器工作目录：`/home/ubuntu/projects/smart-community`，源码放在其中的 `src/`。Gazebo 相机传感器在该容器中实际发布了 640 × 480 RGB 图像；本机的 Nvidia 图形运行时可供它使用。
- 本次基线**不依赖** Ultralytics 权重。YOLO 及车牌 OCR 的训练和接入是后续任务，不能将本版 HSV 灯色检测说成 YOLO 识别。

在新容器中，至少需要以下系统依赖（这里记录所需包，未在新镜像上重新验证从零安装）：

```bash
apt-get update
apt-get install -y python3-colcon-common-extensions python3-opencv python3-numpy \
  ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor \
  ros-jazzy-ament-cmake-auto ros-jazzy-rclcpp-components ros-jazzy-std-srvs
```

需要能运行 Gazebo Ogre2 相机的图形环境。现有容器已具备，不需要重新建容器。现有环境中的 Nav2 和 SLAM Toolbox 是后续接入用的软件，本版启动不依赖它们。运行命令前需要 source ROS 环境；仓库中没有模型权重或额外 GPU 推理依赖。

## 编译和运行

进入**已经创建**的容器，然后执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/projects/smart-community
colcon build --symlink-install --packages-select sentry_motion_manager community_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch community_demo demo.launch.py gui:=true
```

启动后窗口中的粉色顶盖小车会由右上方沿道路巡航，预计约两分钟完成一圈。只运行服务器时指定 `gui:=false`；需要先看清起点再启动时指定 `paused:=true`，然后点击左上方三角形播放按钮。另开容器终端查看：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /patrol/status
ros2 topic echo /perception/traffic_light
ros2 topic hz /camera/image
ros2 topic hz /scan
ros2 topic echo /model/patrol_bot/ground_truth
```

完成时终端出现 `COMPLETE physical single lap; travelled=... m`，小车返回右上方并停下。**停在终点后不会自动循环**。保留窗口再跑一圈：

```bash
ros2 service call /patrol/restart std_srvs/srv/Trigger '{}'
```

独立验收：先以 `paused:=true` 启动，在另一个已 source ROS 环境的终端、仓库根目录执行 `python3 tools/verify_physical_lap.py`，再点击播放。它会检查 10 个路点、物理行驶距离超过 8 m、X/Y 轨迹跨度均超过 2.5 m、倾角始终低于 20°，以及三辆车的扫描距离与几何位置一致，生成 `log/physical_lap_evidence.json` 并以退出码表明是否通过。查看一条“完成”日志不能代替此验收。

已经完成一圈时，可用 `python3 tools/verify_physical_lap.py --restart` 在保持窗口的同时启动下一圈并自动验收。命令必须在仓库根目录运行。它不会在一圈行驶途中强行重置机器人。

`Ctrl+C` 停止 launch；关闭 Gazebo 时也会联动退出 ROS 节点。若需要重建静态世界文件，先在仓库根目录运行 `python3 tools/generate_world.py`，然后重新编译 `community_demo`。

## 系统数据流

```text
Gazebo 物理世界位姿 ─ ground_truth ───→ community_patrol
Gazebo DiffDrive ── wheel odometry ───→ 轮里程计诊断
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

机器人出生位姿为世界坐标 `(1.35, 1.55, π)`。`ground_truth` 由 Gazebo 的物理实体位姿生成，直接在世界坐标系中与路点比较。DiffDrive 的轮里程计与这个话题分开；轮里程计变化不等于底盘真实位移。

路线按示意图的拓扑顺序经过上侧道路、西侧道路、中间横道、中间纵道、下侧道路和东侧道路返回起点，共 10 个控制路点。上侧与下侧设停止点：先发布零速度，等待连续 5 条新收到的相机绿色结果，才推进到下一路点；失去视觉结果时保持停止。`vision.py` 使用 HSV 阈值识别本版模型的亮绿色区域，并做 3 帧一致性检查。近距离 `/scan` 障碍会触发停车，但尚不具备绕行规划。

## 当前边界

1. **已实际验证**：现有容器内两包编译通过；Gazebo 发布相机、雷达、轮里程计和物理世界位姿；底盘真实完成 11.53 m 单圈，两处灯位停靠放行。巡航验收以 `verify_physical_lap.py` 输出为准。
2. **仿真灯目前固定绿色**，还没有红/黄/绿定时切换，因此不能用这次演示证明“红灯不越线”。
3. 人偶是蓝色/橙色立牌、车牌是蓝色矩形占位模型；尚未贴用户提供的 16＋2 人偶与三张车牌素材，也没有对应的 YOLO 数据集、人数统计或 OCR。
4. 目前用 Gazebo 物理世界位姿和固定路点巡航，没有建图、真实定位、Nav2 路径规划、动态避障或车道线越界检测。它是可运行的工程骨架，**不是可直接提交的完整复赛作品**。
5. Gazebo 世界的尺寸和物料大方向取自随附任务图；本版道具位置仍是近似值，正式仿真应使用材料实测尺寸重新校准。

## 下一步优化优先级

保留当前能运行的基线，每加入一个能力都复跑真实轨迹验收。以下功能尚未实现，顺序按当前风险和依赖安排，不代表赛事官方评分顺序。

| 顺序 | 优化项 | 当前问题 | 建议做法及验收 |
| --- | --- | --- | --- |
| 1 | 动态信号灯与停止线 | 固定绿色只能证明看见绿灯后放行，尚未验证红灯停车。 | 加入红/黄/绿切换；用红灯开局、临近停止线变灯、画面丢失等用例检查车头是否越线。 |
| 2 | 场地尺寸与车辆占用范围 | 道具是近似摆放，前方扇区检查没有覆盖完整车身；窄处需要核查侧向间距。 | 按资料校准尺寸与贴图，用实际底盘外轮廓检查整条轨迹与障碍物的间距，再定停车阈值和限速。 |
| 3 | YOLO 检测、人员统计与车牌 OCR | 当前只有适用于本仿真颜色的 HSV 灯色识别，没有人偶/车牌检测。 | 导入真实素材，采集不同位置与角度的图像；按场景分离训练/验证集，训练检测器后接 OCR，并核对类别召回、重复计数和完整车牌正确率。 |
| 4 | 建图与定位 | 目前直接读取 Gazebo 真实坐标，无法移植到未知地图或真机。 | 接入 SLAM Toolbox 或已知地图定位，再接 Nav2；将当前真值仅保留为评估参考，测定位误差和丢失定位后的停车行为。 |
| 5 | 路径跟踪和障碍绕行 | 固定路点、大转角原地旋转、遇障停车；还没有重规划。 | 引入差速底盘路径控制与局部代价地图，先低速验证转角和狭窄通道，再测试临时障碍物绕行；同时检查不能越过车道边界。 |
| 6 | 操作界面与证据输出 | 当前主要依靠终端状态、仿真窗口和 JSON 验收结果。 | 增加当前路段、速度、灯色、识别框、轨迹和故障原因显示，并把识别结果与对应图像保存为可核查记录。 |

当前建议首先完成前两项，再引入训练模型与完整导航。模型大小、训练轮数和导航参数应依据实际数据与测量结果确定。

## 打包

`tools/package.sh` 将源码、世界、README 和来源说明压成 ZIP，排除 `build/`、`install/`、日志和训练数据。复赛官方工程 ZIP 上限为 150 MB；本版代码远低于该上限。训练权重和采集数据以后应单独管理，再按赛事提交要求决定哪些放入工程包。
