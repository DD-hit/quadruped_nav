# 新版导航核心的 Isaac 全流程演示

实际运行链路：Isaac PhysX 步态与射线雷达 / IMU → FAST-LIO → 已有地图点云配准与 `map → odom` → `jie_3d_nav` 全局规划 → `SCAN-Planner` 局部规划与原生速度控制 → `/cmd_vel` → 独立 Isaac 步态接收端。导航和定位不订阅仿真真值。

演示先等待定位请求，再发布粗略初始位姿 `(0.35, -0.20, 0.12)`，等待实际配准确认，最后调用标准 `NavigateToPose` 和 `FollowWaypoints` 接口。包含 1 个单点目标和 3 个连续目标，最后检查速度归零。

## 复现

在服务器工程根目录 `/home/rl/workspace/quadruped_nav` 中：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 tests/run_isaac_navigation_demo.py \
  --project-root /home/rl/workspace/quadruped_nav \
  --output /data0/rl/quadruped_nav/runs/新的运行目录
```

仅在无其他同域仿真、定位或导航节点时运行；演示使用服务器现有的 `ROS_DOMAIN_ID=42` 和本机 DDS 配置，结束后关闭自身启动的进程。

视频由本次保存的物理机身 / 关节状态和 ROS 事件离线回放生成，不重新计算轨迹，不在回放中推进物理仿真。初始位姿请求和定位确认各停帧说明；其余按 1 倍仿真时间播放。

```bash
/home/rl/isaacsim/python.sh tests/render_isaac_navigation_demo.py \
  --project-root /home/rl/workspace/quadruped_nav \
  --session /data0/rl/quadruped_nav/runs/已通过的运行目录 \
  --output /data0/rl/quadruped_nav/runs/新的视频目录
```

## 仿真参数与边界

- 独立演示配置将原生位置纠偏增益 `kp_pos` 设为 `2.0`，使现有步态能响应接近目标时的小速度指令；原示例的 `0.8` 曾在约 `0.18 m` 误差处收敛不足，该中断记录保留。
- 原生角速度上限设为 `0.3 rad/s`，与现有仿真接收端声明的上限一致。
- 位置到达容差仍为 `0.15 m`，朝向容差仍为 `0.15 rad`；任务超时放宽到 `600 s` 墙钟时间，适应仿真低于实时的运行速度。
- 这些覆盖值只保存到会话目录的 `navigation.yaml`，不改动通用导航配置，也不代表 D1 真机标定值。
- 模拟雷达为几何与时序代理，使用逐点射线与内置 IMU；当前射线实现跳过所有机器人自身碰撞体，尚不复现机身和机械臂的真实遮挡。它不复现 Mid-360S 官方扫描花纹、强度和噪声。
- 生成记录可验证仿真闭环与接口；真机仍需核实 SDK 坐标原点、传感器外参、机器人外廓和底层速度响应。

接入时修复了一个外部启动配置路径：`lio_config.py` 读取现有 `fast_lio/config/mid360_nuc_mapping.yaml`，不再依赖已清理的 That-nav 参考目录。


## 2026-11-12 验证结果

`navigation_core_isaac_demo_20261112_01` 已完成粗略初始位姿、实际地图配准、1 个单点目标和 3 个连续目标，最终 `/cmd_vel` 为零。4 个目标的记录位置误差约为 `0.044 / 0.114 / 0.142 / 0.125 m`，均在原 `0.15 m` 到达容差内。

本轮实际保存的 18 条 B-spline 参考轨迹最高速度约为 `0.19973 m/s`；已核对没有跌倒、初始化后瞬移和速度接收端拒绝指令。

独立物理记录核对中，原生双圆柱外廓的最小余量约为 `0.0192 m`。该数字只验证当前原生外廓模型在本次固定场景中的轨迹，不能作为完整机身 / 机械臂外廓或真机鲁棒避障验收。真机外廓、误差裕量和速度响应仍需实测。

- 原生重定时回归：`/data0/rl/quadruped_nav/runs/navigation_recorded_retiming_20261112_01/verification.json`
- 理想接口回归：`/data0/rl/quadruped_nav/runs/navigation_fixed_integration_20261112_01/verification.json`
- 理想绕障回归：`/data0/rl/quadruped_nav/runs/navigation_fixed_avoidance_20261112_01/verification.json`
- Isaac 全流程：`/data0/rl/quadruped_nav/runs/navigation_core_isaac_demo_20261112_01/verification.json`
- 独立物理核对：同一目录的 `independent_evaluation.json`
