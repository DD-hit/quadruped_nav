# quadruped_sim

独立的 Isaac 模型、运动和雷达/IMU 数据源。导航核心不导入 Isaac API，也不依赖步态策略。

根目录 scripts/run_motion.py 提供理想运动/传感器环境；scripts/run_gait_cmd_vel.py 提供独立 D1 仿真速度接收端。与真实 D1 驱动一样，消费 /cmd_vel。

旧导航探针已删除。历史仿真结果保存在数据目录，不作为新导航核心的验收证明。
