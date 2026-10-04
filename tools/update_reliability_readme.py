#!/usr/bin/env python3
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];F=ROOT/'validation/reliability-2026-10-04';S=json.loads((F/'summary.json').read_text());assert S['passed']
r=S['laps'][0];b=S['baseline_normal_lap'];time_delta=r['completion_sim_s']-b['completion_sim_s']
block=f'''## 2026-10-04 可靠性与转弯保护（当前版本）

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
| 完成时间，仿真秒 | {b['completion_sim_s']:.2f} | {r['completion_sim_s']:.2f} |
| 位置误差最大值，cm | {b['position_error_max_m']*100:.2f} | {r['position_error_max_m']*100:.2f} |
| 位置误差P95，cm | {b['position_error_p95_m']*100:.2f} | {r['position_error_p95_m']*100:.2f} |
| 位置误差RMSE，cm | {b['position_error_rmse_m']*100:.2f} | {r['position_error_rmse_m']*100:.2f} |
| 航向误差最大值，度 | {b['yaw_error_max_deg']:.2f} | {r['yaw_error_max_deg']:.2f} |
| 灯柱最小保守净空，cm | {b['minimum_post_clearance_m']*100:.2f} | {r['minimum_post_clearance_m']*100:.2f} |
| 推理+OCR+标注P95，ms | {b['processing_ms_p95']:.1f} | {r['processing_ms_p95']:.1f} |

双角度确认、弯前减速和到达灯相位改变了耗时，本版本正常圈比前版增加{time_delta:.2f}秒。不得把这轮改造描述为总速度提升。固定场景成功不证明未知人物、任意地图或实车性能。

### 三圈实际验收

| 轮次 | 场景 | 仿真秒 | 最大误差cm | 检查结果 |
|---|---|---:|---:|---|
'''
for x in S['laps']:
 block+=f"| {x['lap_id']} | "+{1:'正常自主巡航',2:'识别/定位/巡航暂停，另一次行驶中识别暂停',3:'迟绿灯启动、车牌识别暂停7秒、实际盒子挡路/移除'}[x['lap_id']]+f" | {x['completion_sim_s']:.2f} | {x['position_error_max_m']*100:.2f} | 通过 |\n"
block+='''
每圈17个路径点、8个观察视角、8名居民+2名访客、3车牌正确，车身零越界采样、红灯无跨线、前沿与后沿均在绿灯过线；终端文字与保存图像逐帧核对通过。15个人物观察均有双角度实际反馈。实际节点暂停后停车并恢复；7秒识别暂停触发车牌观察重试，实际障碍移除后重规划并继续。

绿灯统计从接近路口开始：正常/重试圈未多等已见证的新绿灯周期。到达时已经绿灯且不知道开始时刻，等待下一次可确认过渡仍属设计行为；真实视觉断流也可能要求下一周期。默认红10/绿15/黄3秒，不承诺任意短绿灯。

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

'''
p=ROOT/'README.md';s=p.read_text();s=s.replace('## 2026-10-03 红绿灯等待优化（当前版本）','## 2026-10-03 红绿灯等待优化（前版本）');idx=s.index('## 2026-10-03 红绿灯等待优化');s=s[:idx]+block+s[idx:];p.write_text(s)
print('Updated README from completed physical acceptance')
