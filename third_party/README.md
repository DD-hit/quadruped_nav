# 上游来源

导航核心固定使用 jie_3d_nav 与 SCAN-Planner/ros2-community，版本见 upstream.repos。原始代码位于 reference，不参与 colcon 构建；实际迁入源码和必要修改记录在 quadruped_navigation/migration_sources.json 与 docs/source_changes.md。

FAST_LIO_ROS2、Livox 消息和对应补丁保留为外部数据源依赖，不是第三套导航算法。原 That-nav 参考代码与旧导航阶段快照已归档。

## 首次克隆的依赖

third_party/src/FAST_LIO_ROS2 与 livox_ros_driver2 是项目实际使用的小型源码快照，已随仓库提交；ros2_ws/src 中保留相对符号链接。FAST-LIO 的固定来源见 fastlio_source_manifest.json，运行补丁见 patches/，Livox 目录只构建已有消息接口。其他下载依赖、构建结果、地图和录包不纳入 Git。
