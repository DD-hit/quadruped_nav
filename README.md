# quadruped_nav

> 当前状态（2026-11-17）：38 项受控 ROS 接口与动态地图检查通过；当前物理全流程尚未通过验收，尚未完成真机验收。见 [本轮修复与测试](docs/navigation_dynamic_fix_20261117.md)。

机器狗导航项目。导航核心组合 `jie_3d_nav` 与 `SCAN-Planner/ros2-community`，对外提供 Nav2 标准任务接口。当前接入型号为 D1 EDU，目标是迁移到真机。

## 核心分工

| 功能 | 实现 |
|---|---|
| PCD 转 OctoMap、地图保存加载、全局 A* | jie_3d_nav |
| 局部地图、轨迹优化、连续重规划、闭环速度控制 | SCAN-Planner |
| 坐标适配、单点/多点、取消、反馈和结果 | 本项目的薄接口层 |

导航输出 `/cmd_vel`。定位、TF、里程计和点云由外部提供；导航包不启动 D1 SDK、Isaac Sim 或步态策略。

`quadruped_navigation` 是导航核心，`quadruped_interfaces` 保存必要消息。现有模型、传感器、定位和平台适配包独立保留，不参与额外的导航决策。

## 编译和启动

服务器工程：`/home/rl/workspace/quadruped_nav`。

```bash
cd /home/rl/workspace/quadruped_nav/ros2_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
colcon build --packages-up-to quadruped_navigation
source install/setup.bash
ros2 launch quadruped_navigation navigation.launch.py \
  map_package:=/实际导航地图目录
```

启动前，由外部系统提供 `/odom`、`/lio/cloud_registered` 及对应 TF。默认使用真实时间；仿真时设置 `use_sim_time:=true`。

## 使用和配置

- [导航接口与命令](docs/navigation.md)
- [来源与必要修改](docs/source_changes.md)
- [真机接入边界](docs/hardware.md)

主要参数只有一份：`ros2_ws/src/quadruped_navigation/config/navigation.yaml`。速度、机器人外廓和到达容差是原生规划器/控制器参数，示例值不代表 D1 真机已标定。

## 验证范围

历史版本曾通过原生轨迹重定时、理想 ROS 输入接口 / 绕障测试，以及 Isaac 物理步态、模拟雷达 / IMU、FAST-LIO 与地图定位组成的单点 / 多点闭环测试。验证和复现见 [仿真演示说明](docs/simulation_demo.md)。尚未完成真机验收。当前仿真执行端存在零平移指令转向时的机身漂移，详细结果见 [跟踪检查](docs/tracking_review_20261112.md)。

旧实现、脚本与历史文档已移出主工程，备份位于 `/data0/rl/quadruped_nav/runs/navigation_core_before_cleanup_20261111_01`。已有地图、录包、机器人模型和外参数据保留。

## 人工初值定位一键入口

运行 `bash scripts/start_manual_localization.sh`，在 Web 中设置机身位置和朝向，等待定位成功后再导航。使用现有 FAST-LIO + 上游 FAST_LIO_LOCALIZATION2 同源 ICP；不启用全图自动搜索。完整参数与真机迁移边界见 [docs/manual_initialization.md](docs/manual_initialization.md)。
