# 指定赛场的自主建图、地图导航与动态避障

## 实现范围

本实现对应复赛资料的 4.2×4.2 m 指定场地。机器人从规定起点沿赛题路线自主运动，不接受人工遥控，不改变机器人位姿。建图是新启动的 SLAM Toolbox mapping 进程，不加载旧 SLAM 网格或旧图优化图。已有车道示意、已知起点和相机地面标线匹配作为先验保留；这不是未知环境的前沿探索，也不是 AMCL 全局重定位。

## 技术链

1. Gazebo 差速驱动提供实际轮式里程计；相机、云台编码器、激光点云和 50 Hz IMU 经 ros_gz_bridge 进入 ROS 2 Jazzy。轮式增量与相机标线匹配负责 XY，源时间对齐的 IMU 相对朝向负责航向；相机不能再次把 IMU 航向拉偏。已知起点朝向用于初始化。
2. 激光点云生成原始 `/scan`，碰撞保护和 Nav2 障碍层均使用原始扫描。建图专用 `/scan_slam` 只将明确的正无穷无回波替换为 5.99 m；Karto 阈值 5.95 m，传感器最大量程 6 m，因此只更新射线内自由空间，不制造末端障碍。有限障碍距离保持原值，原始扫描不改动。源码依据为 [SLAM Toolbox Jazzy 的 Karto OccupancyGrid::AddScan](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/jazzy/lib/karto_sdk/include/karto_sdk/Karto.h)，此参数仅针对本机 6 m 传感器。
3. 实时 `/map` 保存为 `slam.pgm/yaml`，图优化数据保存为 `slam-graph.posegraph/data`。由于低矮细灯柱的命中可能被后续无回波清掉，额外累积原始有限激光末端：按扫描源时间 TF 转入 field，至少三个不同有效扫描帧命中同一 2 cm 栅格才确认为物理占用。一个扫描帧内重复末端只计一次。
4. `slam-field.pgm` 合成实时 SLAM 栅格和上述确认命中。`navigation.pgm` 再单独施加题目允许车道及 2 cm 障碍安全扩展。未知车道栅格保留未知，禁止用“未知当自由”填补覆盖率。物理占用导出不读取 SDF 灯柱坐标；既有巡航灯柱保护仍使用公开固定场地几何，这个独立保护没有删除。
5. 导出的 `navigation.yaml` 由 Nav2 map_server 加载到 `/field_map`，NavFn A* 计算路径；本项目巡航控制器与 RM motion manager 按路径和受约束目标控制差速底盘。使用 `navigation_mapped.yaml` 启用 static/obstacle/inflation 三层，原始激光实时标记和清除障碍。导航阶段 `slam:=false`，避免把运行中的 SLAM 网格误认为已经加载的保存地图。
6. 路上有碰撞体障碍进入时停车等待；移开后必须有五帧不同源时间的有效清障扫描，重新规划再继续巡航。受限道路内没有证明对长期占道障碍主动绕行。

## 新鲜度与失效保护

IMU 拒绝无效四元数、不可用朝向、非有限数、过期、未来和重复源时间。IMU 最近数据超过 150 ms 时停止输出导航位姿，并发布质量丢失；巡航依照已有定位保护停车。短时间角速度补偿限制在源时间 ±50 ms。相机、轮式里程计和激光的原有新鲜度门槛保留。Gazebo IMU 使用默认理想朝向，本次不证明真实硬件磁干扰、偏置和噪声情况下的精度。

## 验收方法

`tools/run_autonomous_mapping.py` 冻结全部 src 文件和 tools 的 Python/Shell 源码清单及 SHA256，依次重启独立世界执行：

- `autonomous-mapping`：空 SLAM、自动完整巡航、实时地图增长、保存网格和图优化数据。
- `mapped-navigation`：新世界、关闭 SLAM、加载刚保存的导航地图、逐栅格验证实际 map_server 输出、检查所有移动目标均有规划，以及道路、灯态和完整任务。
- `mapped-dynamic`：同一保存地图、有碰撞体的 14 cm 障碍横向进入及移开；读取实时代价地图证明障碍真的被标记和清除，独立车身轮廓审核停车、净空、恢复及整圈。

每圈依旧要求 17 路点、8 次观察、8 社区人员+2 非社区人员、3 个正确车牌、全车身绿灯通过、道路边界合规、终端/事件图像对应、结束静止。实际移动目标是 15 个，因为两个 STOP 路点与前一接近点坐标相同；这不减少 17 路点的物理任务检查。正常位置误差门槛 6 cm、灯柱净空 5 cm、视觉 8 Hz、图像年龄 P95 150 ms 没有放宽。传感器扰动圈沿用原有 12 cm 误差门槛和断流期间误差上界方法。

最终聚合首次遗漏了只读的 sensor_stress_acceptance 真值观察者而报错；核对其 on_truth 只更新停车统计速度、receive 不读取该真值后，修正白名单。原始失败汇总及冻结的旧汇总代码留档；聚合报告记录旧/新分析器哈希，其他运行文件全部逐一保持原冻结值。生产门槛没有调整。

加入 IMU 后，另外重跑行驶中节点暂停、带噪声/丢消息/两秒断流、较暗照明三个旧能力用例。控制使用传感器；仿真真值和实际灯态仅用于独立审计。四个已失败候选保留：两次定位漂移、一次薄障碍网格丢失、一次新地图巡航误差超过门槛；最终结果只统计冻结后的独立复跑。

地图内四根灯柱附近是否有测得占用，用已知坐标在验收阶段检查。报告是“最近确认激光命中格”的距离，不是整张地图的重建精度、柱体语义识别或任意地图泛化准确率。

## 在现有容器运行

先进入 `aic-development`，工作目录 `/home/ubuntu/projects/smart-community`：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/vision_env.sh
source tools/ros_transport.sh
mkdir -p validation/autonomous-mapping-new
"$AIC_VISION_PYTHON" tools/check_mapping_sensors.py --output validation/autonomous-mapping-new/sensor-regression.json
"$AIC_VISION_PYTHON" tools/check_mapping_grid.py > validation/autonomous-mapping-new/grid-regression-corrected.json
"$AIC_VISION_PYTHON" tools/check_optimization_controls.py --output validation/autonomous-mapping-new/control-regression-final.json
"$AIC_VISION_PYTHON" tools/check_boundary_safety.py --output validation/autonomous-mapping-new/boundary-regression-final.json
"$AIC_VISION_PYTHON" tools/run_autonomous_mapping.py --output validation/autonomous-mapping-new/final-01
"$AIC_VISION_PYTHON" tools/run_simulation_refinement.py --output validation/autonomous-mapping-new/regressions-final-01 --cases moving-faults,sensor-stress,dim-light
"$AIC_VISION_PYTHON" tools/summarize_autonomous_mapping.py --output validation/autonomous-mapping-new/final-01 --regressions validation/autonomous-mapping-new/regressions-final-01
```

这条流水线带物理巡航/地图检查和录像，需为每次运行使用全新目录。它不启动桌面 GUI。可手动通过以下命令查看现场建图与 RViz：

```bash
bash tools/launch_full.sh gui:=true rviz:=true
```

这条命令启动新 SLAM；默认导航仍使用原有赛题地图。需要巡航使用验收保存地图时：

```bash
bash tools/launch_latest_mapped.sh gui:=true rviz:=true
```

该入口优先选择本机 `acceptance-summary.json` 已通过、PGM 哈希匹配的保存地图。源码仓库同时附带 `src/community_demo/maps/accepted-2026-10-06/`，没有本机验收目录时加载这里的固定场地验收地图；加载前核对全部网格和 YAML 的 SHA256。它是实际建图结果的副本，不是本次运行重新建出的地图，也不证明未知环境能力。其他目录可以显式指定：

```bash
bash tools/launch_full.sh gui:=true rviz:=true slam:=false \
 navigation_map:=/absolute/path/to/navigation.yaml \
 navigation_config:=/home/ubuntu/projects/smart-community/src/community_demo/config/navigation_mapped.yaml
```

RViz 的 field 固定坐标系显示实时地图、加载地图、障碍代价地图、实际激光、定位和规划路径。图像视频仍只保留本机。`mapping.mp4` 是真实相机+实时网格的连续合成记录，不是 RViz 桌面屏幕录像；`patrol.mp4` 是实际 Gazebo 俯视相机+机器人相机的连续巡航。正式综合比赛视频和队伍答辩段另外完成，不能把短验收片冒充完整提交。
