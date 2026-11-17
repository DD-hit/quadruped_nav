# 人工初值定位：启动和使用

本项目采用 FAST-LIO 连续里程计 + FAST_LIO_LOCALIZATION2 同源的多尺度点到点 ICP 地图匹配。用户提供地图中的大致机身位置和朝向，不启用全图自动搜索。

## 仿真一键启动

在服务器执行：

```bash
cd /home/rl/workspace/quadruped_nav
bash scripts/start_manual_localization.sh
```

该入口自动加载 ROS 2 Humble 和项目安装环境，启动 D1 仿真（默认 GPU 1）、FAST-LIO、定位、现有导航和 Web。所有仿真进程使用同一 DDS 配置。默认地图为 d1_full_room_web_20261117_01。命令按 Ctrl+C 会关闭本次创建的进程。

本地电脑建立转发：

```bash
ssh -N -L 8766:127.0.0.1:8766 rl@100.73.76.90
```

浏览器打开 http://127.0.0.1:8766。HTTP 就绪不表示传感器已经就绪；等待界面显示输入就绪后再设置初值。

操作：在 OctoMap 上选择机身初始位置，指定朝向，点击发送初始位姿；机器人初始化时保持静止。等待定位成功，再选单点或多点目标并开始导航。匹配失败时重新设置初值，不会自动搜索全图，也不会因为发出了初值消息就立即解锁导航。

初值必须是机械 base_link 在 map 下的位姿，不是雷达原点。Web 根据支持面和机身高度填写 Z；ROS 接口使用 PoseWithCovarianceStamped /initialpose，frame_id=map。实际确认状态为 /localization/status，正常 phase=tracking、initialized=true；/localization/pose 发布机身位姿，唯一新增 TF 为 map -> odom。FAST-LIO 仍提供连续里程计，雷达安装外参不被重复应用。

## 常用选项

```bash
# 只检查地图和配置，不启动任何运行进程
bash scripts/start_manual_localization.sh --check-only
# 复用正在运行的仿真；该命令仍创建 FAST-LIO
bash scripts/start_manual_localization.sh --reuse-sim
# 同时复用已有仿真与 FAST-LIO
bash scripts/start_manual_localization.sh --reuse-sim --reuse-lio
# 指定地图、GPU 或 Web 端口
bash scripts/start_manual_localization.sh --map-id d1_full_room_web_20261117_01 --gpu-index 1 --port 8766
```

不要同时启动多个相同 ROS 域、同一 map -> odom 的定位器。需要复用时使用相应选项。改变 Web 端口时，SSH 转发也要相应修改。

## 真机迁移边界

```bash
bash scripts/start_manual_localization.sh --mode real --map-id <真机采集的地图编号>
```

real 模式只启动地图定位节点，复用已经运行的真实雷达、FAST-LIO 和项目机身里程计适配器；不启动仿真、SDK 电机控制或当前仅限仿真的 Web 控制。可从 RViz 或已有 ROS 前端发送 /initialpose。它不是完整真机驱动安装命令，也不表示真机验证已完成。

仿真和真机复用同一套定位接口及雷达外参配置；真实点云的时序、坐标和自遮挡仍需现场验收。定位模型不会读取仿真真值。

## 上游实现说明

直接参考 Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2 的 ros2 分支，固定 commit f04974907c8da976dd0495b272d18ac4c534d41f。现有项目原本也通过 That-nav 使用同源 ICP 方法。

scripts/fast_lio_localization2_core.py 提取配准核心；scripts/map_registration.py 保留已有的地图金字塔、几何与质量检查，并正确地把粗配准结果传给精配准。没有直接搬入上游整个 ROS 节点，因为项目需要已有同步、机身外参、状态和 Web 接口。

使用项目的小房间分辨率和迭代参数，没有把上游默认 5 m 粗匹配距离直接当成通用最佳值。此接入不等于精度必然高于原来；许可证、固定源码和差异见 quadruped_localization/NOTICE、migration_sources.json 和 third_party/reference/fast_lio_localization2。

离线初值容差、实际 Web/Isaac 测试结果见 evaluations/manual_localization_20261117；不要把有限样例的通过数当成任意位置、任意初值保证。任意错误初值识别和全局自动重定位不属于当前功能。
