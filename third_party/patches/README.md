# 本阶段上游变更记录

基线：real-lsy/That-nav，38b12d8d83d10ad1fe1e6b354a34fbf717ae42f4。

- 两个 Livox .msg 原样复制，类型名不变。
- 只为消息提取包替换 CMakeLists.txt/package.xml，移除 SDK 与驱动构建依赖；对照差异见 livox_messages_only.diff（为提取目录对照，不是整个仓库的一键补丁）。
- FAST-LIO preprocess.h/preprocess.cpp、laserMapping.cpp、IMU_Processing.hpp 均原样留档；验证构建额外强制包含 omp.h，不修改原始源文件。
- 上游传感器配置不直接覆写。项目 lio_config.py 从上游基线生成本次 ROS 参数：仿真时间、话题、虚拟单通道/关闭特征、滤点比例、盲区与内部外参。

验证：7 个包构建通过；原始预处理编译通过；18 项实际 ROS 输入验收通过。


## 完整 LIO 集成

`fastlio_runtime.patch` 对固定上游 FAST_LIO_ROS2 包应用：明确 CMake 依赖和 C++17、移除未使用的 Python/pcl_ros 构建依赖，补充 OpenMP 头，增加输出目录参数并路由全部调试/PCD 路径，修正当前协方差的发布顺序。
`IMU_Processing.hpp`、`preprocess.cpp`、`ikd_Tree` 和滤波工具头文件原样保留；common_lib.h 只改变日志路径宏。

仿真启动脚本另外支持可选的每帧射线数覆盖，用于本阶段 6000 条射线验证；默认 1200 条射线的原接口仍通过 12 项回归。
