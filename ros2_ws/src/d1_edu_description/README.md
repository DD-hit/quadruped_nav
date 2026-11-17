# d1_edu_description

已接入提供的 D1 EDU 完整装配。URDF 是模型源，STL 数字单位为米。
25 个连杆、24 个关节，其中 12 个腿部活动关节；SO101 为固定外观。

urdf/d1_edu.urdf 内含设计说明；meshes/ 为原始网格；provenance.json 记录哈希；
config/preview.yaml 定义检查姿态。SDK 机身原点关系仍未核验。

模型原始厂商来源未知。几何和运动学检查不等于接触、惯量或步态验证。

当前固定姿态整机轮廓已测量并与实际 USD 顶点核对：约 0.6993 × 0.3763 × 0.7558 m，包含固定 SO101 与雷达附件。派生约束在 config/footprint.yaml 和 footprint_measurements.json，原 URDF/网格不变。只适用于当前固定姿态平面仿真，历史测量资料已归档；当前导航使用上游参数模型，接入边界见工程 docs/hardware.md。
