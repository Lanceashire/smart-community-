# 人物板正面、侧面和背面的检查

正式世界使用 10 个单面立牌，A/B 街区各五个。图案只印在局部 +X 面，原 PNG 用在对应 Collada 网格上；厚 2 mm 的纸板背面为灰白色。它们沿示意图朝向摆放，不能为了让任意相机视角都看到人物而把图案印到背面或把全部人物转向同一条道路。

例如 A_person_4 使用 resident_03，朝北；从 A 街区南侧看它会看到空白背面，从北侧看才会看到人物。A_person_2 使用 visitor_01，朝西；南侧会主要看到薄边。B_person_3 使用 resident_06，朝东；从北侧看会主要看到薄边。每个模型的贴图及尺寸可核对 `worlds/layout.json` 和 `models/community_assets/manifest.json`。

在现有 ROS/Gazebo 容器中先停止自主巡航，再运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/ros_transport.sh
ros2 launch community_demo demo.launch.py gui:=false patrol:=false vision:=false slam:=false
```

另开同一容器终端，加载相同环境并执行：

```bash
python3 tools/check_person_board_views.py --output log/person-board-views-new
```

该工具拒绝存在 `/patrol/restart` 的场景，从每个模型的印刷面法向两侧拍摄；检查实际机器人位置、云台编码器回正和新鲜相机帧，保存 20 张原始 PNG 与带源时间和 SHA256 的 report.json，最后恢复起点。每次需使用新输出目录。

这是一项通过机器人位姿重置取得的显示检查，只能证明这些视点的实际渲染，不是自主导航或泛化识别率。自主巡航使用真实双角度观察、身份跟踪投票和整圈去重统计，独立于本工具。两张缺失原素材仍是车辆背景和黄灯暗态，与人物贴图无关。
