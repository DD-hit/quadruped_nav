# quadruped_localization

导航的外部定位与建图数据提供者，保持独立。保留 FAST-LIO 输入适配、机身里程计、定位地图管理和地图配准代码。

根目录入口：run_lio.py、run_localization.py、run_mapping.py、save_map.py、load_map.py、set_initial_pose.py。每个脚本可用 --help 查看参数。check_livox_bag.py 与 replay_livox_bag.py 用于录包检查和回放。

导航核心接收它输出的定位/TF、机身里程计和点云，不依赖本包的内部实现。名义外参与仿真噪声参数不能替代真机标定，见工程 docs/hardware.md。历史验收资料已归档。

## 人工初值工作流

使用项目根目录 `scripts/start_manual_localization.sh`。输入 /initialpose 是 map 下的机械 base_link 位姿；定位成功后才允许导航。直接上游参考为 Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2，固定版本和许可证见 NOTICE。匹配核心位于 scripts/fast_lio_localization2_core.py，项目保留坐标、时间同步和质量检查。详见项目 docs/manual_initialization.md。
