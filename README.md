# AIC 智慧社区复赛：场景、巡检导航与视觉识别

基于用户提供的复赛 ZIP，还原 **4.2×4.2 m 场地、60 cm 道路、A/B 人物区、两处动态信号灯和三处车牌**。保留原来 RoboMaster 项目的运动管理与限幅 PID，在 ROS 2 Jazzy / Gazebo Harmonic 中实现单圈巡检。

本工程使用资料中的纸板人像与车牌立牌。人物并非需要行走的行人；它们是机器人巡检识别的目标。场景中的机器人具有有效的驱动轮、支撑结构和运动控制，完成情况由独立物理位姿记录核验。

## 当前验证状态

- 人物：A/B 每处 5 个，逐项位置、方向箭头依据与无箭头时的假设见 `worlds/people_reference.json`。8 个停车观察画面覆盖不同朝向的人像和三辆车牌。
- 传感器：1280×960 单目相机，带一个可转动偏航关节；一台 720×16 多线激光，过滤地面和自身回波后生成 720 点 `/scan`。
- **最终联合验收通过**：17 个路点、实际运动 12.535 m；相机/轮速融合定位最大位置误差 0.0468 m，最大航向误差 6.78°。
- **整车道路边界违规采样 0**；两次停止线均在绿灯通过，见 `validation/final-navigation.json`。这是本次固定布局整轮验收，不代表所有变灯时间与布局都已覆盖。
- **人员 8 居民 + 2 访客**，身份列表与参考布局完全一致，未多计或漏计。
- **车牌三张**：苏A·B8Q62、鄂D·7B5Q2、苏A·PL12A，均由相机裁切 OCR 读出并多帧确认。
- 终端事件按识别顺序输出；同一事件画面、时间戳与编号已冻结在本机 `validation/final-evidence/`。按用户要求，验收 JPG 只保留本机交付压缩包，GitHub 保留文字记录与事件 JSON。独立视觉评分见 `validation/perception-audit.json`。
- 实际激光 SLAM 地图见 `maps/slam.pgm`、`maps/map-evidence.json`，来源为 `/map`。
- 新增 Hugging Face 权重实测与经授权微调的 YOLO11s 候选：同一扩充测试集 268 张上，nano 的 22 类 mAP50 / mAP50–95 为 **84.08% / 67.91%**，S 为 **89.89% / 78.50%**。原 nano `best.pt` 完整保留，S 单独保存为 `yolo11s-comparison.pt`，可用启动环境变量选择。详见下方比较协议；这些分数不代表真实社区或比赛成功率。

**尚缺的原素材：** ZIP 内 `车牌背景.png.baiduyun.p.downloading`、`黄灯暗.png.baiduyun.p.downloading` 不能解码。按用户要求暂缓替换，保留生成的薄车头板和暗黄灯几何模型。其余 27 张原图保持原始字节，哈希写入素材清单。

## 赛题要求与对应实现

| 资料要求 | 实现 |
| --- | --- |
| 尺寸和示意布局 | 原图地面贴图、按尺寸生成 SDF/Collada、人像位置朝向对照表 |
| 自主一圈巡检 | 巡航任务状态机 → Nav2 `ComputePathToPose` → 北极熊限幅 PID 差速跟踪 → Sentry26 运动管理 → Gazebo |
| 建图与定位 | 多线激光 → 过滤后的扫描 → SLAM Toolbox 建图；轮速增量 + 相机地面标线配准输出导航位置 |
| 人物识别与统计 | YOLO11n 的 18 个资料人像身份类别；多帧确认，按视觉身份去重，再统计居民/访客 |
| 车牌位置及字符 | YOLO 车牌检测 + 蓝色背景边界补检/修框，选择中央车牌后 RapidOCR 识别，多帧确认 |
| 红灯停绿灯行 | 相机灯态检测、连续帧确认、进入停止线前再次检查灯态新鲜度；识别不确定时停止 |
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

控制节点只订阅 `/localization/pose`，不订阅 `/model/patrol_bot/ground_truth`。后者只供独立评估与离线数据标注工具使用。

### 道路规划与停止线

`maps/roads.pgm` 是根据资料的可行驶道路建立的语义占据栅格；它不是场景中的假墙。Nav2 NavFn 使用 A* 生成路径，巡航实际跟踪返回的路径。全局规划使用静态道路约束，近距离障碍物由真实激光回波触发停止；当前不实现绕过临时堵路后自动重规划。

轮廓采用 **22 cm × 22.5 cm**，宽度包含轮子外侧，而不是只看机器人中心点。运行时预测 0.1/0.3/0.5 秒位置并检查轮廓采样；评估工具使用实际物理位姿重复检查。红灯规则在等待点确认绿灯，并在车头尚未越过停止线时检查灯态变化，信息失效或变红会再次停止。

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
