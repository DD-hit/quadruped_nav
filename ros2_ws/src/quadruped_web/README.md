# quadruped_web

项目自己的 Web 上位机。第一版为仿真调试版本，复用现有 jie_3d_nav + SCAN-Planner 的 ROS 2 导航接口，PCD 用于定位，OctoMap 用于三维显示与现有全局规划。

启动前 source ROS 2 Humble 和工作空间，使用项目 scripts/run_web.py。HTTP 仅绑定服务器 127.0.0.1，通过 SSH 本地端口转发访问；不向公网开放 ROS。

地图包保留原始地图坐标；显示高度不影响规划。OctoMap 实际叶节点尺寸用于渲染。目标来自已有 traversable 层，支持多高度表面选取。初值将支撑表面高度转换为 nominal base 高度，参数尚未真机标定。

导航页：地图加载、图层/高度裁剪、初值、单点/多点位置与朝向编辑、顺序调整、Nav2 Action 下发和取消/停止、真实任务反馈与路径。手动页：显式获取控制权、按住运动、速度限幅、松开/失焦归零、服务端超时保护。摄像头、SDK 预设动作、地图点位管理页、系统诊断页明确保留待开发。

速度链：trajectory_controller -> /navigation/cmd_vel_raw -> quadruped_web_bridge -> /cmd_vel。后台是此会话唯一速度出口，不修改规划或轨迹跟踪算法。控制输入超时归零，浏览器控制租约超时取消任务。外部任务取消采用标准 Action CancelGoal 服务。存在另一个 /cmd_vel 发布源时本入口归零并报冲突（无法阻止外部发布者本身）。

已知限制：当前初始化可能接受错误初值，tracking 不代表位置正确，导航前要求人工核对。现有地图 layers 不等于完整整机外廓/真机通行验收。模拟雷达不含真实自身遮挡。真机 SDK 及超时保护未接入，--mode real 明确拒绝启动运动控制。不能把仿真测试结果当成真机验收。
