# quadruped_navigation

全局规划来自 jie_3d_nav；局部规划、重规划和速度闭环来自 SCAN-Planner。

入口：`ros2 launch quadruped_navigation navigation.launch.py`。定位、TF、里程计和点云由外部提供，输出 `/cmd_vel`。

任务使用 Nav2 标准 Action。详见工程 `docs/navigation.md`。
