# 已验收固定场地地图副本

来自 2026-10-06 的自主建图实跑，PGM/YAML 原字节复制，哈希见 manifest.json。navigation 是施加赛题允许车道约束后的导航网格；slam 是原始实时 SLAM 网格；slam-field 是加入反复实测激光末端并注册到场地坐标系的网格。

`bash tools/launch_latest_mapped.sh` 在没有本机验收输出时加载这里的 navigation.yaml。需要重新自主建图时运行 tools/run_autonomous_mapping.py 并使用新输出目录。此副本只适用于随附固定场地及起点，不代表未知地图导航。
