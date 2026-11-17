# Web 上位机 0.1

当前为 D1 EDU 的仿真调试版本。正式代码位于 `/home/rl/workspace/quadruped_nav/ros2_ws/src/quadruped_web`，不再是本地示意网页。界面保持已确认的浅色风格。

## 访问与启动

本机通过 SSH 隧道访问 `http://127.0.0.1:8766/`。原 `8765` 是旧的纯样例。

```bash
ssh -N -L 127.0.0.1:8766:127.0.0.1:8766 rl@100.73.76.90
```

服务器新终端启动 Web、定位与导航（请勿重复启动已运行的会话）：

```bash
source /opt/ros/humble/setup.bash
source /home/rl/workspace/quadruped_nav/ros2_ws/install/setup.bash
/usr/bin/python3 /home/rl/workspace/quadruped_nav/scripts/run_web.py --with-lio --output /data0/rl/quadruped_nav/runs/web_$(date +%Y%m%d_%H%M%S)
```

仿真仍属于 quadruped_sim，独立终端启动；不要同时运行其他 /cmd_vel 发布源：

```bash
source /opt/ros/humble/setup.bash
source /home/rl/workspace/quadruped_nav/ros2_ws/install/setup.bash
export QUADRUPED_SIM_CAPTURE_RGB=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rl/workspace/quadruped_nav/ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml
/usr/bin/python3 /home/rl/workspace/quadruped_nav/scripts/run_gait_cmd_vel.py --standby-on-stop --gpu-index 1 --max-seconds 600 --output-dir /data0/rl/quadruped_nav/runs/web_sim_$(date +%Y%m%d_%H%M%S)
```

退出 Web 会话会取消任务、清零指令并关闭自己启动的定位/导航进程；独立启动的仿真需在对应终端退出。当前在线展示的仿真设为最多 600 s 仿真时间，约几十分钟墙钟时间，退出后页面会如实显示传感器/位姿超时，不会伪装在线。

## 导航页

- 一个真实 OctoMap 三维视窗，鼠标左键旋转、右键平移、滚轮缩放。
- 地图包选择在标题下方，小窗口也可操作；旧图层预览包显示为待核对且不可选。
- 占据、可通行、禁行、风险层来自已有地图包；实时点云来自实际 /navigation/cloud。
- 高度范围在地图右上方，支持输入与滑块双向调整。当前室内地图可先将上限设为 1.50 m，方便查看房间内部。只影响显示，不修改地图或规划。
- 初始位姿：点击可通行表面，拖动朝向，弹窗可以微调机身 X/Y/Z 与朝向，确认后发送 /initialpose。需等待 FAST-LIO 完成初始化且机器人静止；跟踪中仍需人工核对位置与朝向。
- 单点/多点：选择表面、拖动朝向；右侧点位可编辑坐标、朝向、上移、下移、删除。
- 开始导航：弹窗核对目标与地图，勾选已经核对定位后确认下发。使用 NavigateToPose / FollowWaypoints，收到后端接受才进入执行状态。
- 执行期间点位锁定，显示当前航点；右侧“取消任务”取消 Action。地图上方“停止”同时请求停止与任务取消。“指令已停”表示控制器 IDLE、速度指令为零，并不保证有惯性的物理机身瞬间静止。
- 全局路径、局部 B-spline 轨迹与定位历史只展示实际算法输出，不生成虚假路径。刷新时恢复后台正在执行的任务点位。

## 手动控制页

- 点击“启用手动控制”获取控制权。自动任务未停止时会拒绝，不会混合两套速度输出。
- 按住前进/后退/左移/右移与左右旋转，松开归零；W/A/S/D、Q/E 同样可用，空格停止。
- 只有方向区中央一个小停止按钮，没有重复的大停止按钮。
- 平移、旋转速度可调；对角移动合成速度受限。显示的是下发速度，不是实际机身速度。
- 松开、移出按钮、失焦、切页清除手动输入；服务端手动指令超时 0.3 s 清零，浏览器控制租约超时 5 s 退出控制并取消任务。
- 摄像头、截图/全屏、SDK 站立/趴下/恢复/回正动作明确显示待接入且禁用，不伪造画面或动作成功。

“地图与点位”和“系统诊断”两个完整管理页面继续保留待开发入口。

## 地图与运行边界

新地图包 `/data0/rl/quadruped_nav/maps/d1_full_room_web_20261113_01` 组合已验证运行保存的原生 OctoMap/图层和原始定位 PCD，保持相同 map 坐标。PCD SHA256 与原始地图一致。OctoMap 按真实叶节点尺寸渲染，共 29,067 个占据叶节点。

未修改现有定位、全局规划、局部规划或轨迹跟踪算法。新增模块只做 Web、接口适配和速度控制权仲裁：原生 controller → /navigation/cmd_vel_raw → quadruped_web_bridge → /cmd_vel。

错误初始化被接受的问题尚未修复；地图图层不是完整整机/真机通行证书。SDK 原点与速度响应、真实传感器噪声及自身遮挡未真机验收。首版 --mode real 明确拒绝启动，真机接入前需要校准参数并接入机器人侧控制接口与超时保护。

## 验证

最终全链路：`/data0/rl/quadruped_nav/runs/web_isaac_acceptance_20261113_04/http/verification.json`，19 项 PASS。
控制模块：`tests/test_web_control.py`，7 项 PASS。
包括真实 OctoMap/PCD 配对、显式初始化、单点到达、两航点顺序到达、取消和停止确认、手动速度输出及位姿运动、多客户端互斥、超时归零、租约过期、地图切换清除定位，以及未实现能力不伪装。

另检查包编译、Python/JavaScript 语法、浏览器实际三维显示/高度输入/点选/任务反馈。启动端口冲突会明确拒绝。导航源文件哈希与 2026-11-12 验证版本一致。

开发期间失败记录保留：早期初始化请求过早、ROS 2 结构化点云解析兼容、以及测试目标不在原有可通行层。修复接口和测试输入后重跑通过；没有通过放宽定位/规划门限掩盖失败。
