# AIC 智慧社区复赛：场景、巡检导航与视觉识别

基于用户提供的复赛 ZIP，还原 **4.2×4.2 m 场地、60 cm 道路、A/B 人物区、两处动态信号灯和三处车牌**。保留原来 RoboMaster 项目的运动管理与限幅 PID，在 ROS 2 Jazzy / Gazebo Harmonic 中实现单圈巡检。

本工程使用资料中的纸板人像与车牌立牌。人物并非需要行走的行人；它们是机器人巡检识别的目标。场景中的机器人具有有效的驱动轮、支撑结构和运动控制，完成情况由独立物理位姿记录核验。

## 场景与已有验证

- 人物：A/B 每处 5 个，逐项位置、方向箭头依据与无箭头时的假设见 `worlds/people_reference.json`。8 个停车观察画面覆盖不同朝向的人像和三辆车牌。
- 传感器：1280×960 单目相机，带一个可转动偏航关节；一台 720×16 多线激光，过滤地面和自身回波后生成 720 点 `/scan`。
- **早期固定场景基线验收通过**：17 个路点、实际运动 12.535 m；相机/轮速融合定位最大位置误差 0.0468 m，最大航向误差 6.78°。
- **整车道路边界违规采样 0**；两次停止线均在绿灯通过，见 `validation/final-navigation.json`。这是本次固定布局整轮验收，不代表所有变灯时间与布局都已覆盖。
- **人员 8 居民 + 2 访客**，身份列表与参考布局完全一致，未多计或漏计。
- **车牌三张**：苏A·B8Q62、鄂D·7B5Q2、苏A·PL12A，均由相机裁切 OCR 读出并多帧确认。
- 终端事件按识别顺序输出；同一事件画面、时间戳与编号已冻结在本机 `validation/final-evidence/`。按用户要求，验收图片仅保留本机，不进入 GitHub 或当前工程交付压缩包；GitHub 保留文字记录与事件 JSON。独立视觉评分见 `validation/perception-audit.json`。
- 实际激光 SLAM 地图见 `maps/slam.pgm`、`maps/map-evidence.json`，来源为 `/map`。
- 新增 Hugging Face 权重实测与经授权微调的 YOLO11s 候选：同一扩充测试集 268 张上，nano 的 22 类 mAP50 / mAP50–95 为 **84.08% / 67.91%**，S 为 **89.89% / 78.50%**。原 nano `best.pt` 完整保留，S 单独保存为 `yolo11s-comparison.pt`，可用启动环境变量选择。详见下方比较协议；这些分数不代表真实社区或比赛成功率。

**尚缺的原素材：** ZIP 内 `车牌背景.png.baiduyun.p.downloading`、`黄灯暗.png.baiduyun.p.downloading` 不能解码。按用户要求暂缓替换，保留生成的薄车头板和暗黄灯几何模型。其余 27 张原图保持原始字节，哈希写入素材清单。

## 2026-10-04 可靠性与转弯保护（当前版本）

本轮重点是人物证据一致性、实测云台角度、传感器断流恢复和弯前保护。**原YOLO11s权重保留，离线检测精度没有新的提升声明。**

### 具体变化

- 人物按框关联轨迹，补偿小角度云台运动；多帧身份投票后去重，抑制同一轨迹因标签波动重复计数。
- 人物观察取得两个真实`camera_pan_joint`角度：第一角度至少5稳定帧、第二至少3帧、角差至少0.05rad。第一角度完成条件由实测帧反馈触发，反馈须匹配同一观察尝试。
- 直线最高0.22m/s，弯前24cm、航向误差较大和终点附近限速0.12m/s；角速度上限0.55rad/s。
- 位姿保留最后里程计源时间，超过0.15s的旧样本不作为新定位发布或驱动依据；全程视觉消息/图像过期停车。
- 新增GPU推理、OCR、模型、地图、构建产物启动检查；识别进程退出时关闭ROS/Gazebo，停止识别进程有明确超时。
- 默认`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，避免实测进程暂停恢复时原有数据流挂起的SHM路径；全部传感器桥接流使用best-effort小队列。内部DDS根因未取得栈证据，当前结论来自症状与更换传输后的实跑回归。[Fast DDS官方配置](https://fast-dds.docs.eprosima.com/en/v2.14.7/fastdds/env_vars/env_vars.html#fastdds-builtin-transports)。

### 正常圈前后对照

| 指标 | 前版本正常首圈 | 本版本正常首圈 |
|---|---:|---:|
| 完成时间，仿真秒 | 151.80 | 181.20 |
| 位置误差最大值，cm | 3.01 | 2.86 |
| 位置误差P95，cm | 2.47 | 2.19 |
| 位置误差RMSE，cm | 1.33 | 1.10 |
| 航向误差最大值，度 | 6.82 | 6.06 |
| 灯柱最小保守净空，cm | 4.96 | 5.55 |
| 推理+OCR+标注P95，ms | 20.8 | 19.9 |

双角度确认、弯前减速和到达灯相位改变了耗时，本版本正常圈比前版增加29.40秒。不得把这轮改造描述为总速度提升。固定场景成功不证明未知人物、任意地图或实车性能。

### 三圈实际验收

| 轮次 | 场景 | 仿真秒 | 最大误差cm | 检查结果 |
|---|---|---:|---:|---|
| 1 | 正常自主巡航 | 181.20 | 2.86 | 通过 |
| 2 | 识别/定位/巡航暂停，另一次行驶中识别暂停 | 214.15 | 10.88 | 通过 |
| 3 | 迟绿灯启动、车牌识别暂停7秒、实际盒子挡路/移除 | 202.65 | 2.39 | 通过 |

每圈17个路径点、8个观察视角、8名居民+2名访客、3车牌正确，车身零越界采样、红灯无跨线、前沿与后沿均在绿灯过线；终端文字与保存图像逐帧核对通过。15个人物观察均有双角度实际反馈。实际节点暂停后停车并恢复；7秒识别暂停触发车牌观察重试，实际障碍移除后重规划并继续。

绿灯统计从接近路口开始：正常/重试圈未多等已见证的新绿灯周期。到达时已经绿灯且不知道开始时刻，等待下一次可确认过渡仍属设计行为；真实视觉断流也可能要求下一周期。默认红10/绿15/黄3秒，不承诺任意短绿灯。

另补录一圈正常演示并独立核对通过：183.55仿真秒，最大位置误差3.21cm；完整图像缓冲接口录像留本机，避免读取半写入磁盘JPEG。该圈不含故障注入。

### 模型微调候选：拒绝替换

使用原736张训练图，仅重复弱类训练样本到1344条目，12轮微调，没有新增独立图片；验证558张不变。mAP@0.5:0.95从81.48%到82.50%，访客1召回从53.54%到55.74%，车牌召回从82.29%降到74.86%。未达到预设替换门槛，保留原YOLO11s；候选没有再用测试集反复挑选。原测试268张：P91.73%、R85.06%、mAP50 89.89%、mAP50:95 78.50%；访客1召回55.81%仍偏弱。

### 复现

在已配置GPU的现有容器：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sentry_motion_manager community_demo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash tools/launch_full.sh gui:=false paused:=true
```

另一个容器终端：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/ros_transport.sh
python3 tools/audit_patrol_cycles.py --laps 3 --fault-lap 2 --unpause --restart-phases red,green --green-delay 10 --timeout 500 --output validation/new-audit
```

另按目标圈同时执行`probe_perception_motion.py --lap 2`、`probe_observation_retry.py --lap 3`、`probe_obstacle_recovery.py`，分别指定新输出文件；真实录像可用`record_camera_acceptance.py --lap 3 --output log/new-lap3.mp4`。各探针需要在目标阶段开始前启动；审计结束后才运行汇总，不能以临时未完成JSON作结论。

换机器：宿主运行`bash tools/build_image.sh`生成基础镜像，Dev Container配置使用本机HOME变量；容器先`bash tools/bootstrap_vision.sh`建立项目`.venv-vision`，默认PyTorch2.7.1/cu128与torchvision0.22.1，再安装OCR依赖并构建ROS。也可显式设置`AIC_VISION_PYTHON`。当前机器使用既有只读PyTorch2.13/cu130环境，干净新机器配方尚未独立重建，不声称两套环境已经同等验收。

本轮证据：`validation/reliability-2026-10-04/summary.json`、`METHOD.md`、`runtime-hashes.json`、`public/`和各故障探针。四次开发尝试均留存，其中包括未确认观察、OCR变量崩溃、旧里程计时间戳和暂停恢复数据流挂起。它们不加入最终成功圈数。完整采样随本机工程包；图片、视频及含截图的材料草稿仅留本机，不上传GitHub。队伍信息和两张缺失素材由用户明确暂缓。

## 2026-10-03 红绿灯等待优化（前版本）

修复了接近停止线时已经等过红灯，进入正式停止点后却清空记录、再等一轮的问题。`top_approach/top_stop` 共用上方路口相位记录，`lower_approach/lower_stop` 共用下方路口记录；换路口、完赛或重新开圈才清空路口上下文。运行时只使用相机灯态，不读取仿真灯色话题或周期倒计时。

放行仍须两次红／黄灯证据和连续五次绿灯确认。新增按仿真时间记录的绿灯起始时刻：从首次看到新绿灯起，4 秒内允许确认放行，避免接近途中耽搁后使用旧绿灯记录；短暂 unknown 不会刷新这个窗口。灯态消息中断超过 0.7 秒或时间倒退会清空记录。已有停止线保护、新鲜度检查和整车轮廓保护继续生效。

**最终三轮完整物理验收通过，六次路口通过均在该次接近过程中观察到的第一轮新绿灯放行，额外错过的绿灯周期为 0。** 每轮 17 路点、8 个确认视角、8 居民＋2 访客和三块正确车牌；道路越界采样为 0，车头进入和车尾清线均在绿灯。终端与图像对应检查也通过。

| 条件 | 一圈完成仿真秒 | 上方新绿灯至放行秒 | 下方新绿灯至放行秒 | 上／下额外绿灯周期 |
| --- | --- | --- | --- | --- |
| 红灯阶段首圈 | 151.80 | 1.55 | 1.55 | 0 / 0 |
| 绿灯末段重新启动 | 158.75 | 1.55 | 1.75 | 0 / 0 |
| 红灯阶段再次启动 | 151.15 | 1.50 | 1.55 | 0 / 0 |

历史重复等待首圈在上方路口第一次新绿灯出现后 29.55 秒才放行；本轮首圈为 1.55 秒，消除了约一个 28 秒灯周期的额外等待。对应历史整圈 179.80 秒，本轮首圈 151.80 秒；这是各一轮的观测对照，不能把不同启动灯态的圈次混成统一加速比例。

边界：此处“一轮绿灯通过”指已观察到非绿→新绿变化后，不再无故跳过这轮绿灯。第一次看到灯就已经是绿灯、且不知道它亮了多久时，仍会等下一次可确认的变化；不能承诺任何到达时刻都直接通过。当前场景红／绿／黄为 10/15/3 秒，4 秒放行窗口已在这套时序验收，未保证任意更短绿灯周期。

证据：`validation/signal-optimization-2026-10-03/summary.json` 含前后对照、六次绿灯周期和整车过线时刻；`controller-checks.json` 覆盖实际回调的路口切换、状态继承、消息断流与窗口过期。历史源码保存在 `before-source/`；完整逐帧记录随本机工程包交付。世界、YOLO11s 权重与本次导航参数未改变，验收图片仅留本机。

复现时备份已有该目录的 `final/` 后，关闭其他社区世界，在容器工程目录执行：

```bash
# 终端 1
bash tools/launch_full.sh gui:=false paused:=true
# 终端 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/check_signal_handoff.py
python3 tools/audit_patrol_cycles.py --laps 3 --fault-lap 0 --unpause \
  --restart-phases green,red --green-delay 10 --timeout 350 \
  --output validation/signal-optimization-2026-10-03/final
python3 tools/summarize_signal_waits.py
```

聚合程序使用本机工程包内的历史原始记录作对照；GitHub 保存汇总数值和工具。测试工具读取仿真真值与灯态用于独立评分，控制节点不订阅这些评分输入。

## 2026-10-03 性能与跑图验收（前版本，10 月 2 日采集）

**最终代码五轮物理仿真验收通过：3 轮正常巡航、1 轮节点暂停恢复、1 轮出生偏差叠加物理障碍清除恢复。** 每轮完成 17 路点、8 个确认观察方向、8 居民＋2 访客、3 块正确车牌；整车道路边界违规采样均为 0，两处路口的车头进入和车尾清线均在绿灯。终端事件与本机保存图像帧号逐项核对通过。

### 相同标称起点的前后对照

| 指标 | 优化前 | 优化后 | 变化 |
| --- | --- | --- | --- |
| 一圈完成仿真时间（不含结束后审计停车） | 191.35 s | 179.80 s | 缩短 6.04% |
| 行驶 GO 状态累计 | 105.70 s | 96.70 s | 缩短 8.51% |
| 观察 OBSERVE 状态累计 | 44.55 s | 24.70 s | 缩短 44.56% |
| 最大新鲜定位误差 | 5.92 cm | 2.84 cm | 降低 52.00% |
| 定位误差 95 分位 | 3.02 cm | 2.37 cm | 降低 21.42% |
| 定位误差 RMSE | 1.89 cm | 1.13 cm | 降低 40.13% |

这是各一轮的直接对照，并非多轮统计显著性结论。基准开着 Gazebo GUI，最终自动验收关闭 GUI、保留相机渲染，因此主要对比仿真时间；墙钟完成时间 193.259 / 181.723 秒仅作参考。红灯等待实测从 33.8 增至 44.1 仿真秒，故减少观察时间不能等比例换算成整圈收益。最终控制器的防护停车也计入总耗时。

### 实际改动

1. **缩短有充分视觉证据的停留。** 人物最短观察 3 秒、车牌 2.5 秒；还须至少处理 6 帧、观察满 1.8 秒、证据集合稳定 0.8 秒且反馈新鲜，才允许提前结束。未确认仍执行原有两次补拍，超过 13.5 秒明确记录未确认，不用预置答案补齐。
2. **修正定位输入。** 按图像时间匹配实际 `camera_pan_joint` 编码器角度；只在相机回正且稳定时做地面白线配准，防止把侧向立牌白色区域误当道路。长时间没有有效地面校正时先减速，再停车等待局部观测恢复；巡航和 RM 运动管理统一使用仿真时钟。
3. **修复转弯碰柱的根因。** 原规划栅格遗漏信号灯立柱，可能使路径切入碰撞区，导致轮子打滑和里程计偏差。现在从 SDF 提取 4 根立柱，补入 `roads.pgm` 与 `planning-obstacles.json`；运行时增加整车预测矩形与立柱的碰撞检查。右下转弯路点向路内调整 4 cm。场景实物、人物和官方地面均未移动。
4. **直道提速、恢复后重规划。** 直道上限由 0.18 提至 0.22 m/s，接近停车点、较大转角和定位退化时降速。近障先停车，连续 5 次激光确认清障后重新请求合法路径；长期阻塞明确报告。当前实现为清障后恢复，尚不具备动态障碍绕行。
5. **统一默认模型。** `launch_full.sh` 默认使用已验证的 YOLO11s，nano 权重仍完整保留。本轮没有训练或改变权重，离线 mAP 没有提高；S 的既有 268 图测试 mAP50 / mAP50–95 仍为 89.89% / 78.50%。

### 五轮结果与边界

| 条件 | 完成仿真时间 s | 最大新鲜定位误差 cm | 结果 |
| --- | --- | --- | --- |
| 标称出生位置，正常首圈 | 179.80 | 2.84 | 通过 |
| 识别／定位／巡航节点分别暂停约 2.5 s | 156.25 | 17.20 | 停车并恢复，通过 |
| 绿灯中途重新启动 | 186.85 | 2.73 | 通过 |
| x/y ＋2.5 cm、yaw ＋0.035 rad，叠加物理堵路 | 154.70 | 4.61 | 清障后恢复，通过 |
| x/y −2.5 cm、yaw −0.035 rad | 150.70 | 4.09 | 通过 |

不同启动相位的红灯等待不同，不能用最快一圈代表相同条件的加速。故障圈的 17.20 cm 瞬时误差保留在结果中，不能将正常圈 2.84 cm 的精度外推到定位中断时。五轮对立柱的最小保守净空为 3.25 cm；独立审计使用真值，运行控制不读取真值位置。

物理堵路测试在机器人前方生成真实碰撞方块：机器人停车后距方块前缘约 13.9 cm，稳定速度为 0；移除方块后确认清障、重新规划，从移除到到达 y>1 m 的目标检查线约 3.65 秒。这个时间包含继续行驶，不是规划器耗时。测试没有瞬移机器人、发布替代速度或注入识别答案。

当前结论限于资料的固定场地、已知起点及上述小幅出生偏差。尚未验证任意起点全局重定位、陌生布局、真实相机和动态绕障。识别弱类的既有问题仍在，例如 visitor_01 测试召回率约 55.81%；完整路线识别通过不等于所有视角都能可靠识别。

### 证据与复现

汇总见 [`validation/performance-2026-10-02/summary.json`](validation/performance-2026-10-02/summary.json)，方法见 [`METHOD.md`](validation/performance-2026-10-02/METHOD.md)。`public/` 保存不含图片的逐轮数值核验和开发失败记录；完整采样在本机同目录 `baseline/`、`candidate/`、`start-plus/`、`start-minus/`。验收图在本机 `log/performance/acceptance-performance.png`，不上传。`final-source-manifest.json` 固定验收控制代码、地图、世界及模型哈希；`baseline-source/` 保留前版本关键代码。

开发中失败与中止尝试保存在 `attempt-*`：包括时钟不一致、云台角度输入问题、暂停世界和遗漏立柱导致的碰撞。它们没有计入最终五轮通过，也没有改写成成功。最终权重和正式世界哈希与前版本一致。

先按下方环境说明完成依赖与 `colcon build`。日常演示在容器工作目录执行：

```bash
bash tools/launch_full.sh gui:=true paused:=false
# 完赛后可在 http://127.0.0.1:8765 页面再次启动一圈
```

复现验收时先备份已有证据目录，再在容器内运行（不要与日常演示并行）：

```bash
# 终端 1，工作目录 /home/ubuntu/projects/smart-community
bash tools/launch_full.sh gui:=false paused:=true
# 终端 2，source ROS 和 install/setup.bash 后
python3 tools/audit_patrol_cycles.py --laps 3 --fault-lap 2 --unpause \
  --restart-phases red,green --timeout 350 \
  --output validation/performance-2026-10-02/candidate
# 主验收通过后运行；将停止主世界，依次运行两种已有出生偏差世界
python3 tools/audit_performance_variants.py
python3 tools/check_performance_controls.py
PYTHONPATH="$PWD/.deps:${PYTHONPATH:-}" \
  /home/ubuntu/.local/share/aic-vision-venv/bin/python tools/finalize_performance.py
python3 tools/export_performance_evidence.py
```

出生偏差世界与清单沿用 `tools/prepare_patrol_start_variants.py` 的产物。重建导航栅格使用 `python3 tools/build_navigation_map.py`，随后重新构建工作区；`tools/save_maps.py` 也使用同一逻辑，避免保存 SLAM 地图时擦掉立柱约束。环境仍为 ROS 2 Jazzy、Gazebo Harmonic、Python 3.12.3、PyTorch 2.13.0+cu130、Ultralytics 8.4.164 和 RTX 5060 Laptop GPU。

## 2026-10-02 巡航优化验收（前版本记录）

优化前的严格检查因下方路口车尾在黄灯清线而未通过。当前控制器在停止点先通过相机观察到至少两次红／黄灯状态，再连续确认五次绿灯才放行；已在绿灯中途到达也等待新一轮绿灯。灯态过期会重新建立该条件。控制节点不使用仿真信号状态或周期倒计时；原来的车身停止线保护仍生效。代价是增加等待时间，以下实际测得正常圈为 195.10–230.70 秒（含结束后停车核验）。当前仿真灯周期仍是红/绿/黄 10/15/3 秒，不能把这些结果外推到任意短绿灯周期。

观察点最低停留约 5.5 秒，使用当前视角的多帧视觉确认反馈。未确认则将相机偏航依次调整 ＋0.08 / −0.08 rad，观察上限约 13.5 秒；仍未确认会记录 `UNCONFIRMED_VIEW`、`confirmed=false` 和 `unconfirmed_views`，界面显示未确认点。人物确认要求本视角存在重复身份证据且无近期待确认候选；车牌要求本视角重复高置信度 OCR。反馈不读取场景答案，不保证发现所有被遮挡或完全未检测到的人物。

最终代码的六圈物理巡航全部通过：5 圈正常运行，1 圈明确标记的故障测试。每圈 17 路点、8 观察方向、8 居民＋2 访客、3 块正确车牌；六圈共 48 个确认观察点和 165 条终端/图片事件。整车道路越界采样 0；12 次路口通过的车头与车尾均为绿灯，红灯跨线停留采样 0。红灯停车时车身距停止线最小约 11.7 cm。

| 测试条件 | 仿真时长 s | 实际距离 m | 最大新鲜位姿差 cm | 严格结果 |
| --- | --- | --- | --- | --- |
| 受控故障＋车牌补拍 | 203.55 | 12.541 | 14.54 | 通过 |
| 绿灯末尾启动 | 230.70 | 12.528 | 7.28 | 通过 |
| 红灯启动 | 195.10 | 12.537 | 5.19 | 通过 |
| 黄灯启动 | 226.90 | 12.554 | 6.28 | 通过 |
| 起点偏差＋2.5 cm / ＋2° | 196.05 | 12.544 | 4.14 | 通过 |
| 起点偏差−2.5 cm / −2° | 195.45 | 12.508 | 7.96 | 通过 |

故障圈分别暂停识别、定位、巡航节点约 2.5 秒，停车并恢复完成；故障期间的约 14.54 cm 位姿差不代表正常定位精度。另外在第一车牌观察点真实暂停识别节点 7 秒，恢复后一次相机补拍，在约 8.9 秒完成确认。重试超时明确报告未确认的分支另以真实控制方法的离线回归验证，未伪造检测标签发布到 ROS。

起点偏差测试只改变 SDF 初始出生位置，运行中不瞬移；相机/轮速定位仍从标称先验初始化。两个变体分别为 x/y 的 ±2.5 cm、航向 ±0.035 rad，不能据此宣称任意起点重定位。场景布局、人物身份、车牌、模型权重与检测阈值未改变，当前验收使用 YOLO11s 候选权重。

详细证据为 `validation/patrol-optimization-2026-10-02/summary.json`，分圈记录在 `main/`、`start-plus/`、`start-minus/`；代码与输入哈希、环境、控制回归和重试探针记录均保留。上一轮黄灯清线失败在 `baseline-summary.json` 原样保留；初步圈重试次数记录问题修正后重新进行了上述最终六圈。验收工具独立读取真实车身轮廓、核对实际终端 JSON、检测框/名称/置信度，并以 OCR 读取保存 JPEG 的 Frame 号。best-effort 大图话题漏收仍只作为诊断。实际轨迹图和 JPG 留在本机 `log/patrol-optimization/` / `log/perception/`，不上传 GitHub。

复现主验收：

```bash
# 宿主机：原容器因临时 Xauthority 路径失效而不能启动时，恢复其显示绑定。
bash tools/start_existing_container.sh aic-development
# 容器终端，先 source ROS 和 install/setup.bash
AIC_PERCEPTION_MODEL=src/community_demo/models/perception/yolo11s-comparison.pt \
  bash tools/launch_full.sh gui:=true paused:=true
# 另一个容器终端，世界仍暂停时启动真实车牌识别暂停/补拍探针
python3 tools/probe_observation_retry.py
# 再一个容器终端，执行主验收
python3 tools/audit_patrol_cycles.py --laps 4 --fault-lap 1 --unpause \
  --restart-phases green,red,yellow \
  --output validation/patrol-optimization-2026-10-02/main --timeout 350
```

`tools/prepare_patrol_start_variants.py` 生成两个仅初始位姿不同的世界，重新构建 `community_demo` 后，`tools/audit_start_variants.py` 在主验收结束后运行两种偏差测试；最后保留已完成、停止的仿真窗口。聚合与实际轨迹图使用 `tools/summarize_patrol_optimization.py`。初次构建/依赖准备需遵循下方环境说明；这些脚本不包含已知人物或车牌结果的运行时注入。

环境实测：Python 3.12.3、ROS 2 Jazzy、Gazebo Harmonic、PyTorch 2.13.0+cu130、Ultralytics 8.4.164、RTX 5060 Laptop GPU。当前仍未覆盖真实相机、陌生身份、任意起点、进程崩溃/GPU 重置或动态障碍物绕行；两项原素材缺失仍按用户要求暂缓。

## 2026-10-02 巡航循环复查（优化前记录）

本次针对实际巡航、再次启动及比赛输出规则复查，使用单独保存的 YOLO11s 候选权重。机器人每轮按指定路径完成后停车，再次启动会开启新的圈次；题目要求的是一轮自主巡检。

已修复每圈观察记录累积、识别状态依赖 GO 文本重置、事件图片重名覆盖、灯态评估提前报告未确认颜色，以及 OCR 置信度显示与标签重叠问题。新增运行 UUID 与圈次编号，完赛后冻结统计；新圈清空人数、车牌、投票和观察记录。实时传感器发布/订阅采用 best-effort 短队列，避免暂停节点对生产数据流施加可靠传输等待；低频圈次消息保留可靠持久发布。

初始两圈正常巡航通过，但暂停定位节点后的故障圈出现长时间 `WAIT_FRESH_LOCALIZATION`，超过 330 秒验收时限；该失败保留于 `validation/patrol-cycle-review/before-recovery-fix.json`。QoS 调整后的同类故障重放完成：识别、定位、巡航节点分别暂停约 2.5 秒，停车并在恢复后继续，额外覆盖下方红灯等待。随后再次完成正常一圈。

| 修复后实测 | 正常重复巡航 | 含故障与下方红灯覆盖的巡航 |
| --- | --- | --- |
| 实际物理距离 | 12.532 m | 12.555 m |
| 记录时长（含完赛后 4 秒停车核验） | 165.75 s | 200.95 s |
| 路点 / 观察方向 | 17 / 8 | 17 / 8 |
| 人物 / 正确车牌 | 8 居民 + 2 访客 / 3 | 8 居民 + 2 访客 / 3 |
| 整车道路边界违规采样 | 0 | 0 |
| 红灯跨停止线 / 跨线停留 | 0 / 0 | 0 / 0 |

红灯等待时，上、下两处停止线距车身最近分别约 15.8 cm 与 12.2 cm。正常圈最大定位误差 5.15 cm；故障圈短暂失去定位更新时，记录到 14.52 cm 的位置差，不能将正常精度外推到数据中断期间。三个节点暂停后，在 1.5 秒核验窗口内速度均已为零，并继续完成剩余路段。

**黄灯边界仍须明确记录：**正常圈下方停止线车头在绿灯 t=584.10 s 越线，车尾在黄灯 t=585.35 s 清线。未观察到红灯越线，但附加的“车头车尾都只在绿灯越线”检查未通过，因此综合严格检查 `summary.json` 的 `passed` 为 false。原任务没有单独规定黄灯清线细则；后续可采用等待新一轮绿灯再起步的更保守策略，不能把这项写成全绿灯清线通过。

两圈共 50 条事件与真实终端 JSON、同帧检测框/名称/置信度匹配；使用 OCR 独立读取保存 JPEG 上的 Frame 数字，全部与事件帧号一致。原始大图 best-effort 订阅漏收了部分事件帧，这个诊断仍保留；持久保存的事件图片用于一一对应验收。图片在本机 `log/patrol-cycle-review/`，不上传 GitHub。

证据：`validation/patrol-cycle-review/final/summary.json`（含附加未通过项）、`lap-001.json`、`lap-002.json`、对应 `*-images.json` 和 `lower-red-probe.json`。原题文字与 ZIP 字节一致性及车身外轮廓检查见 `requirements-and-geometry.json`。

复现时先以 YOLO11s 启动暂停的完整世界，在另外两个已 source ROS/工作区的容器终端运行：

```bash
# 终端 1：制造下方红灯等待覆盖，暂停对象是已在停止线前停车的控制节点。
python3 tools/probe_lower_red_wait.py --lap 1 \
  --output validation/patrol-cycle-review/final/lower-red-probe.json
# 终端 2：两圈验收，第一圈短暂暂停三个节点，第二圈为正常自主巡航。
python3 tools/audit_patrol_cycles.py --laps 2 --fault-lap 1 --unpause \
  --output validation/patrol-cycle-review/final
```

完整世界启动命令为 `AIC_PERCEPTION_MODEL=src/community_demo/models/perception/yolo11s-comparison.pt bash tools/launch_full.sh gui:=true paused:=true`。故障工具只对指定进程发送 SIGSTOP/SIGCONT，异常退出会恢复进程；验收工具只读取真值，不给控制器注入位置或识别答案。所有验收仍限于固定素材、固定布局和已知起点，未覆盖进程崩溃、GPU 重置、任意动态障碍物或陌生人物。

## 赛题要求与对应实现

| 资料要求 | 实现 |
| --- | --- |
| 尺寸和示意布局 | 原图地面贴图、按尺寸生成 SDF/Collada、人像位置朝向对照表 |
| 自主一圈巡检 | 巡航任务状态机 → Nav2 `ComputePathToPose` → 北极熊限幅 PID 差速跟踪 → Sentry26 运动管理 → Gazebo |
| 建图与定位 | 多线激光 → 过滤后的扫描 → SLAM Toolbox 建图；轮速增量 + 相机地面标线配准输出导航位置 |
| 人物识别与统计 | YOLO11n / YOLO11s 的 18 个资料人像身份类别（当前优化验收使用 S）；多帧确认，按视觉身份去重，再统计居民/访客 |
| 车牌位置及字符 | YOLO 车牌检测 + 蓝色背景边界补检/修框，选择中央车牌后 RapidOCR 识别，多帧确认 |
| 红灯停绿灯行 | 相机确认非绿→新绿灯后连续帧放行，进入停止线前检查灯态新鲜度；识别不确定时停止 |
| 不压道路边线 | Nav2 道路栅格约束；含外轮面的整车轮廓预测检查；独立真值评估实际轮廓 |
| 框、名称、置信度及终端顺序 | 同一帧产生带框图像和 JSON，首次确认事件按序打印并保存对应画面 |

## 技术链与语言

- **Python 3.12**：场景生成、定位融合、巡检状态机、视觉、训练和评估。
- **C++ / ROS 2**：复用 Sentry26 运动管理节点，以及已安装的 Nav2、SLAM Toolbox、ROS-Gazebo 桥。
- **SDF / Collada / YAML**：世界、模型材质、地图与 ROS 配置。无需自行编写渲染器。
- YOLO 使用 **GPU**；OCR 使用两线程 ONNX Runtime CPU，以减少与 Gazebo/检测模型的资源争用。

### 定位为什么采用相机和轮速融合

场地没有高围墙，15 cm 人偶较低，单线激光在 23.5 cm 高度会漏掉人物。多线激光向下覆盖目标，并按高度过滤地面。这个稀疏场景直接使用纯激光扫描匹配控制机器人时，实测出现异常与明显误差，因此导航定位改为：

1. 从驱动轮里程计取得增量，按当前估计航向积分。
2. 根据相机焦距、26.5 cm 安装高度、0.10 rad 下俯角和相机偏航，将画面地面白线反投影到地面。
3. 将观测白线与提供的地图白线距离场配准，在轮速预测附近搜索小范围位置和航向修正。
4. 相机或里程计丢失时停止发布新鲜导航位姿，巡航停止。

已知起始位置是任务的开局条件。当前版本不提供任意位置绑架恢复，不能把这次固定场地结果说成通用定位系统。SLAM Toolbox 继续使用滤后的激光和融合里程计生成地图；巡航位置不直接取纯激光匹配结果。

控制器从 `/localization/pose` 和 `/localization/quality` 接收位置与局部定位质量，不订阅 `/model/patrol_bot/ground_truth`。后者只供独立评估与离线数据标注工具使用。

### 道路规划与停止线

`maps/roads.pgm` 是根据资料的可行驶道路和真实信号灯立柱建立的占据栅格；它不是场景中的假墙。Nav2 NavFn 使用 A* 生成路径，巡航实际跟踪返回的路径。全局规划使用静态道路与立柱约束，近距离障碍物由真实激光回波触发停止；连续扫描确认障碍清除后重新规划并继续，当前不实现绕过仍在堵路的动态障碍。

轮廓采用 **22 cm × 22.5 cm**，宽度包含轮子外侧，而不是只看机器人中心点。运行时预测 0.1/0.3/0.5 秒位置并检查轮廓采样；评估工具使用实际物理位姿重复检查。红灯规则从路口接近阶段记录相位，在等待点继承记录并确认新鲜绿灯；在车头尚未越过停止线时检查灯态变化，信息失效或变红会再次停止。

### 视觉身份与统计边界

YOLO 类别为 `resident_01..16`、`visitor_01..02`、`plate`、`light_red/yellow/green`，合计 22 类。居民与访客分类来自资料提供的身份分组，身份需要多帧一致识别后才记一次。

这适用于资料中的固定人像图库。它不是人脸识别系统，也没有证明对新的真实人物能够判断其社区身份。当前布局的参考人数是 8 居民 + 2 访客；这个值只出现在独立评分工具中，运行时统计由检测结果累积，不读取布局答案。

车牌框有部分裁切和漏检时，由蓝色背景连通区域补检并校正边界。图像中 `geom` 值是区域形状评分，不是 YOLO 概率；JSON 分别标明方法、形状评分和 OCR 置信度。该补检适配资料中的蓝牌，尚未覆盖绿牌或其他车牌颜色。

OCR 同样不读取车牌答案；参考字符串只在 `tools/verify_perception.py` 的独立评估中使用。车牌必须符合本场景蓝牌的字符长度与格式，并经至少两次识别确认。

## 模型训练

### 本次数据

数据来自 **Gazebo 机器人相机实际图像**，不是直接把透明 PNG 当作训练照片。单独的 `training.sdf` 放置 18 种身份，用于采集；另有正式布局转头采集批次；赛题 `community.sdf` 保留 10 个参考人物。

- 原始训练集 736 帧、验证集 101 帧、测试集 140 帧。复查发现旧测试集有 4 张与训练集字节完全相同，原先 140 张测试的分数存在泄漏，不能作为严格独立结果。
- 本次再采集验证 480 帧、测试 180 帧；对原图和新增图进行跨集合 SHA256 / pHash 近似去重后，冻结 **train 736 / val 558 / test 268**。训练图片未增加；val 用于选择训练检查点，test 用于最后对比，没有将 val/test 图片加入训练。
- 按采集批次分组：训练随机种子 42/314/514 和一轮自主采集，验证 237，测试 991/719；未把同一段视频的相邻帧随机混入验证/测试。
- 标签由已知仿真几何离线投影生成，并过滤背面与明显投影遮挡。标签预览已人工抽查。这个方法需要继续抽查边缘遮挡，不能声称标签完全没有误差。
- 这些批次仍来自同一素材图库、主要光照与固定采集场景；指标不代表真实相机或全新场景的泛化性能。
- 不使用水平翻转增强，避免把人像和文字镜像化。使用小幅角度、尺寸、亮度变化与 Mosaic。

### 复现训练

在现有容器内：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 首先只启动采集场景，不能同时运行自主巡航。
ros2 launch community_demo demo.launch.py world:=training.sdf patrol:=false
```

另一个容器终端依次采集，**同一世界只能运行一个位姿采集任务**：

```bash
python3 tools/collect_dataset.py --batch train42 --random-views 320 \
  --seed 42 --layout training-layout.json --split train
python3 tools/collect_dataset.py --batch val237 --random-views 100 \
  --seed 237 --layout training-layout.json --split val
python3 tools/collect_dataset.py --batch test991 --random-views 80 \
  --seed 991 --layout training-layout.json --split test
```

采集结束后关闭这个 Gazebo 世界，再训练：

```bash
/home/ubuntu/.local/share/aic-vision-venv/bin/python tools/train_yolo.py \
  --epochs 45 --imgsz 960 --batch 6
```

原 nano 使用官方 YOLO11n 权重初始化；其训练后的权重仍为 `src/community_demo/models/perception/best.pt`。原始采集帧、虚拟环境和训练日志不进入提交包。本机先训练 45 轮，再在补充相机转头视角后基于首轮权重继续训练 25 轮，记录见 `validation/yolo-training*.json`。

## Hugging Face 对比与 YOLO11s 候选

### 数据和评价协议

- 新批次来自 `training.sdf` 与正式 `community.sdf`，分别使用正常、偏暗、偏亮三种光照；变换相机视角、距离和偏航。验证和测试使用独立随机种子，所有 22 类均有实例。
- 离线采集使用静态、无碰撞的相机支架，等待位姿确认、新图像及时间戳对齐后保存；它只用于数据采集，不用于巡航运动验收。原赛题场景不改动。
- 标签由几何投影生成，比较副本修正了 26.7 cm 相机高度和前景黄色机盖遮挡；人工抽查 24 个画面，尚非逐框人工标注。数据仍来自同一资料图库，测试集少数身份仅 8 个实例，需继续扩展与人工核验。
- 对跨集合图片使用 SHA256 和 pHash 汉明距离 ≤2 去重。清单见 `validation/comparison-dataset.json`，类别与光照统计见 `validation/expanded-data-audit.json`。
- 所有检测器使用相同图像、960 输入、batch 8、GPU、置信度起点 0.001、NMS IoU 0.7、max_det 300；mAP50–95 衡量更严格的框位置。模型类别映射后再对合并类别做相同 NMS。
- 原 nano 训练 45+25 轮，使用旧验证集和原始标签；S 经用户明确授权，使用同样的 736 张训练图片及修正后的标签，训练 45 轮、batch 4，使用扩充验证集选 checkpoint。因此这是**可用权重的同测试集比较**，不是仅改变网络尺寸的严格消融实验。

### 完整 22 类结果

| 模型 | 验证 mAP50 | 测试 Precision | 测试 Recall | 测试 mAP50 | 测试 mAP50–95 |
| --- | --- | --- | --- | --- | --- |
| 保留的 YOLO11n | 87.42% | 83.82% | 79.02% | 84.08% | 67.91% |
| 微调后的 YOLO11s | 92.07% | 91.73% | 85.06% | 89.89% | 78.50% |

S 的测试 mAP50 提升 5.81 个百分点、mAP50–95 提升 10.59 个百分点。测试 batch 8 下纯模型推理约 7.13 ms/图，nano 约 3.25 ms/图；这不是机器人端含取图、ROS、绘制与 OCR 的端到端延迟。训练时本机 GPU 显存约 2.9 GB，无显存溢出。

### 现成模型的同图像子任务实测

| 权重 | 人物框 mAP50 | 车牌框 mAP50 | 灯态框 mAP50 |
| --- | --- | --- | --- |
| 当前 nano（身份合并为人物） | 92.35% | 86.16% | 92.64% |
| 本次微调 S（身份合并为人物） | 94.75% | 88.41% | 92.12% |
| [Ultralytics/YOLO11](https://huggingface.co/Ultralytics/YOLO11) 原始 S | 84.52% | — | — |
| [morsetechlab 车牌 YOLO11](https://huggingface.co/morsetechlab/yolov11-license-plate-detection) n / s | — | 60.68% / 72.68% | — |
| [yahyagul 交通标志 YOLOv8](https://huggingface.co/yahyagul/traffic-sign-yolov8) | — | — | 4.30% |
| [dronefreak LISA YOLO26l](https://huggingface.co/dronefreak/lisa-yolo26l) | — | — | 0.16% |

通用 COCO 人物类别不能认出我们的 18 个资料身份；上表的人物指标只是找人框，不能替代完整身份比较。交通灯模型与本题的单颗亮灯、微型立牌及框定义存在差异，低分只说明直接迁移到本数据不适合，不说明其原始任务性能差。车牌仓库作者自己提示其原数据划分有污染，不能拿主页宣传分数直接与我们的分数比。

S 并非每个子任务都更好：灯态总体 mAP50 略低，黄灯/绿灯仍需补数据；测试红灯 mAP50 为 S 95.03% / nano 92.75%，灯态整体 mAP50–95 为 S 81.73% / nano 79.85%。不能声称所有灯色识别都提高。

权重仓库版本与文件 SHA256 已固定在 `validation/hf-model-sources.json`。比较原始结果（含逐类指标和速度）见 `validation/hf-model-comparison.json`、`validation/comparison-results/`，训练记录见 `validation/yolo11s-training.json`。原始数据、模型下载缓存及验收画面只保留本机，不上传 GitHub。

S 候选已经接入 Gazebo 完成两圈固定布局验收：人物 8 居民 + 2 访客、三张正确车牌、两处绿灯通过、道路边界违规采样 0。第一圈完整道路评估见 `validation/yolo11s-navigation.json`；第二圈物理记录为 17/17 路点、实际行驶 12.517 m，见 `validation/yolo11s-physical-lap.json`，视觉评分见 `validation/yolo11s-perception-audit.json`。两圈的记录分别列出，不合并伪称同一次采样。第二圈事件画面冻结在本机 `validation/yolo11s-evidence/`，JPG 不上传 GitHub。

### 运行候选模型

在已有容器、已构建工作区中运行，先关闭正在运行的旧巡航：

```bash
AIC_PERCEPTION_MODEL=src/community_demo/models/perception/yolo11s-comparison.pt \
  bash tools/launch_full.sh gui:=true
```

不设置变量仍使用原 nano，启动脚本不会覆盖权重。S checkpoint SHA256：`f46341d9116fa227995f04c14dcb390508e244f26c109be90a6e9a4ff0ffbefa`。

### 复现比较流程

数据未包含在仓库中，需要先自行采集。离线采集期间只运行一个 Gazebo 世界，并停止自主巡航：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/prepare_validation_worlds.py
python3 tools/run_validation_matrix.py
/home/ubuntu/.local/share/aic-vision-venv/bin/python tools/build_comparison_dataset.py
```

下载固定版本的公开权重，可在有网络代理配置的宿主机工作区运行：

```bash
python3 tools/download_comparison_models.py
```

如需重新训练候选（需要自行决定是否重新训练；默认运行不训练）：

```bash
/home/ubuntu/.local/share/aic-vision-venv/bin/python tools/train_comparison_s.py --epochs 45
```

训练输出位于 `log/training/hf-yolo11s-comparison/weights/best.pt`，训练工具不覆盖项目权重；评估新训练结果前手动将其复制为 `src/community_demo/models/perception/yolo11s-comparison.pt`。对比：

```bash
/home/ubuntu/.local/share/aic-vision-venv/bin/python tools/compare_yolo_models.py --include-s
```

数据清单和评估 JSON 用于复查；扩大验证集能增加发现错误的机会，本身不会自动提高模型准确度。后续应优先补黄/绿灯、少样本身份、遮挡与真实相机图，并保持最终测试集不参与调参。

## 本机已验证环境

| 项目 | 当前值 |
| --- | --- |
| 宿主 | Garuda / Arch，原生 Docker Engine |
| GPU | NVIDIA RTX 5060 Laptop，约 8 GB，驱动 615.71.09 |
| 容器 | `aic-development`；原 `nifty_hugle`、`aic-scene-jazzy` 保留作回退 |
| 系统 | Ubuntu 24.04.4 |
| ROS / Gazebo | Jazzy / Harmonic，Gazebo Sim 8.15.0 |
| GPU 视觉 | PyTorch 2.13.0+cu130，Ultralytics 8.4.164 |
| OCR | RapidOCR 3.9.2、ONNX Runtime 1.30.0 |
| 渲染 | 实测 Zink/Vulkan → NVIDIA GPU；相机帧实测约 8 Hz，配置 10 Hz |

宿主源码：`/home/jiezichuan/Documents/Codex/2026-09-28/x/outputs/smart-community-docker/workspace`。
容器绑定目录：`/home/ubuntu/projects/smart-community`。
GPU 虚拟环境只读挂载：`/home/ubuntu/.local/share/aic-vision-venv`。
额外 OCR 依赖写入工程 `.deps/`，不改变原只读环境。

其他机器应自行提供 NVIDIA Container Toolkit、可显示 Gazebo 的 X11/Wayland-XWayland 桌面，以及上述 ROS 软件包。容器名、DISPLAY、Xauthority 文件属于本机会话，不能照抄为通用常量。

## 编译、运行、验收

在现有容器终端：

```bash
cd /home/ubuntu/projects/smart-community
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sentry_motion_manager community_demo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 只需首次安装 OCR 依赖；已有 .deps 时可跳过。
bash tools/install_vision.sh
```

完整巡航与视觉：

```bash
bash tools/launch_full.sh gui:=true
```

浏览器打开 `http://127.0.0.1:8765`，查看真实相机、检测框和去重结果。页面的“再巡航一圈”按钮可以在完成后重新启动，并清空上一圈统计。识别终端日志在 `log/perception-terminal.log`；事件对应图片、JSON 在 `log/perception/`。

仅观察场景：`ros2 launch community_demo scene.launch.py gui:=true`。
HSV 基线调试巡航：`ros2 launch community_demo demo.launch.py gui:=true`，它不启动 YOLO，不能当作完整视觉验收。
这些模式选择其一，不能启动多个同名 Gazebo 世界。

完成一圈后再次巡航：

```bash
ros2 service call /patrol/restart std_srvs/srv/Trigger '{}'
```

重启会清空上一圈的视觉统计，保留机器人窗口。启动整轮评估可使用暂停开局：

```bash
bash tools/launch_full.sh gui:=true paused:=true
```

另开终端启动以下记录器，再点击 Gazebo 播放按钮：

```bash
python3 tools/verify_physical_lap.py --output validation/final-lap.json --timeout 340
python3 tools/audit_navigation.py --output validation/final-navigation.json --timeout 340
```

两个记录器应分别在终端中运行。完成后：

```bash
python3 tools/verify_perception.py
python3 tools/save_maps.py
python3 tools/check_scene.py
```

## 场景尺寸和摆放假设

- 人物高度约 15 cm 根据资料照片中的尺子估计；宽度保留原图比例；纸板厚 2 mm 为仿真假设。
- 车辆立牌：高 25 cm、宽 34.5 cm、厚 5 mm；车牌 9.5×3 cm。
- 门架：宽 64 cm、总高 48 cm、净空 34 cm、灯板高 14 cm；灯径 12 cm 为模型假设。
- 信号周期红/绿/黄 10/15/3 秒是可修改的仿真默认值；附件任务文字没有指定周期。
- A/B 图标中心按题图边界映射到米制坐标。箭头不明确的个体采用面向相邻巡检道路的假设；资料没有固定每个位置的身份编号。
- 原图含 18 种人物素材不代表必须全部摆入参考场景，参考题图采用 A/B 各 5 人。可更改布局，并应重新验收相机覆盖和视觉计数。

## RoboMaster 代码来源

| 上游 | 本工程使用部分 |
| --- | --- |
| [SMBU 北极熊路径控制器](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) | 限幅 PID 改为 Python 差速路径跟踪，大转角先原地转向 |
| [Boombroke/Sentry26](https://github.com/Boombroke/Sentry26) | 原 C++ 运动管理器，命令限速、加速度限制、超时与急停入口 |

未使用 `Gh0stown/AiC-2026`。上游提交、版权、视觉依赖和素材来源见 `NOTICE.md`、`licenses/`。本项目代码的 Apache-2.0 不替代 Ultralytics AGPL-3.0、字体或用户素材的各自权利声明。

## 打包

```bash
bash tools/package.sh /tmp/smart-community-source.zip
```

提交包包含代码、场景、原素材、训练后模型、OCR 模型、README、来源声明和验证结果；排除虚拟环境、数据集、构建产物与普通日志。附件要求代码包不超过 150 MB，最终包大小和哈希见交付记录。
