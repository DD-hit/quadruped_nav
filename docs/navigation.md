# 导航接口

输入为外部定位后的里程计、已注册点云和 TF；输出为机身坐标系下的速度。接口层组织任务，路径与速度由上游核心计算。

## 数据要求

| 数据 | 默认接口 | 要求 |
|---|---|---|
| 机身里程计 | `/odom`，`nav_msgs/msg/Odometry` | child_frame_id 为 base_link；twist 在机身坐标系 |
| 已注册点云 | `/lio/cloud_registered`，`sensor_msgs/msg/PointCloud2` | frame_id 必须能通过 TF 变换到 map |
| 坐标关系 | `map → odom → base_link → lidar_link` | 外部定位和机器人描述负责提供 |
| 导航地图 | `map_package` 启动参数 | 上游工具保存的 meta.yaml、octomap_msg.npz、layers.npz |
| 速度输出 | `/cmd_vel`，`geometry_msgs/msg/Twist` | vx、vy 单位 m/s，wz 单位 rad/s，均相对机身 |

`frame_adapter` 使用消息时间戳变换坐标，不把雷达原点当成机身原点。局部地图的射线原点由真实雷达 TF 提供，规划器和控制器使用机身位姿。

定位地图与导航地图分别由外部定位模块和本项目地图工具维护，但必须使用同一坐标约定。旧地图文件保留；旧自定义图层不作为额外约束直接接入新执行链，迁移时应确认其上游格式与语义。

## 启动

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
source /home/rl/workspace/quadruped_nav/ros2_ws/install/setup.bash
ros2 launch quadruped_navigation navigation.launch.py \
  map_package:=/实际导航地图目录 \
  odom_topic:=/odom cloud_topic:=/lio/cloud_registered
```

本机既有驱动配置使用 ROS_DOMAIN_ID=42。所有提供输入、运行导航和接收速度的进程需使用相同域号。ROS 接口脚本使用 Humble 对应的系统 Python。

可覆盖的接口参数：`planning_frame`、`base_frame`、`lidar_frame`、`odom_topic`、`cloud_topic`、`cmd_vel_topic`、`params_file`、`use_sim_time`。任务目标使用 planning_frame，默认 map。

## 任务接口

| 名称 | 类型 | 行为 |
|---|---|---|
| `/initialpose` | PoseWithCovarianceStamped | 外部定位模块消费；导航收到时停止当前任务 |
| `/goal_pose` | PoseStamped | RViz 单点目标入口 |
| `/navigate_to_pose` | Nav2 NavigateToPose Action | 单点导航，包含最终朝向 |
| `/follow_waypoints` | Nav2 FollowWaypoints Action | 按顺序到达各点；失败时停止，返回失败点 |
| `/navigate_through_poses` | Nav2 NavigateThroughPoses Action | 有序多点接口；当前按点依次到达，不保证不停顿穿越 |

反馈、结果和取消均使用 ROS 2 Action 标准机制。忙时拒绝第二个任务；取消确认后不会重新执行旧轨迹。自定义 Nav2 行为树不属于这层接口。

初始位姿只是定位输入，导航核心不负责重新定位，也不会移动机器人。外部定位正常提供 map 下的机身位姿和点云后，才接受目标。

## 命令示例

```bash
cd /home/rl/workspace/quadruped_nav
/usr/bin/python3 scripts/navigation.py initial 0.0 0.0 0.0
/usr/bin/python3 scripts/navigation.py goal 1.0 0.0 0.0
/usr/bin/python3 scripts/navigation.py waypoints ros2_ws/src/quadruped_navigation/config/waypoints.example.yaml
/usr/bin/python3 scripts/navigation.py cancel
```

位置单位 m，朝向单位 rad。仿真时 CLI 加 `--use-sim-time`，例如 `/usr/bin/python3 scripts/navigation.py --use-sim-time goal 1.0 0.0 0.0`。示例点位需按实际地图修改。

## 导入和保存导航地图

准备地图时单独运行以下入口，不与导航入口重复启动地图管理节点：

```bash
ros2 launch quadruped_navigation import_map.launch.py \
  pcd_file:=/实际地图.pcd resolution:=0.1
ros2 service call /navigation/map_package_manager/save_package \
  quadruped_interfaces/srv/SaveNavigationMapPackage \
  "{package_path: '/实际输出目录', overwrite: false}"
```

导航入口通过 map_package 自动加载地图，也可在空闲时调用 `/navigation/map_package_manager/load_package`。全局规划服务为 `/navigation/compute_path`，类型 `nav_msgs/srv/GetPlan`；全局路径显示话题为 `/navigation/global_path`。

## 运动方式

运动速度沿 SCAN 的平滑局部轨迹切线，并叠加位置误差反馈。机身朝向根据前方预瞄点调整，不要求始终与瞬时切线重合；角度误差较大时先转向，较小时边走边转，保留原生侧向速度能力。到达后对齐目标朝向。

参考速度 0.2 m/s 不超过示例中较弱的平移轴能力；预瞄时间为 0.4 s。goal_tolerance 与 finish_dist 应保持一致。真机参数仍需按实际平台核对。

没有自定义双模式、逐连杆制动否决层、步态补偿或强制停稳分段执行器。上游碰撞检查和连续重规划仍然保留。
