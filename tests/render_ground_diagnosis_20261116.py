import json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
run=Path('/data0/rl/quadruped_nav/runs/ground_diagnosis_20261116_02');r=json.loads((run/'report.json').read_text());font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');floor=np.load(run/'final_floor_samples.npz')['navigation.pcd'];g=np.load(run/'gravity_series.npz');coef=np.array(r['maps']['navigation.pcd']['best_plane_z_ax_by_c'])
fig,ax=plt.subplots(1,2,figsize=(13,4.5),dpi=160)
a=floor[abs(floor[:,1])<.25];ax[0].scatter(a[:,0],a[:,2]*100,s=3,c='#6d91b8',alpha=.5,label='PCD 地面点（取中央窄条）');xx=np.linspace(-5.7,5.7,100);ax[0].plot(xx,(coef[0]*xx+coef[2])*100,color='#b3814c',label='诊断拟合线，不修改地图');ax[0].axhline(0,color='#7b8d7d',linestyle='--',label='仿真真实水平地面');ax[0].set_xlabel('world X / m');ax[0].set_ylabel('Height / cm');ax[0].set_title('原本水平的地面，在地图中出现倾斜',fontproperties=font);ax[0].legend(prop=font,fontsize=8)
ax[1].plot(g['time_s'],g['tilt_deg'],color='#6d91b8',lw=1,label='FAST-LIO 估计竖直方向与固定地图轴的夹角');ax[1].axhline(r['maps']['navigation.pcd']['best_plane_tilt_deg'],color='#b3814c',linestyle='--',label='最终导航地图地面倾斜角约 0.445°');ax[1].set_xlabel('Mapping time / s');ax[1].set_ylabel('Angle / degree');ax[1].set_title('建图期间，估计的竖直方向发生变化',fontproperties=font);ax[1].legend(prop=font,fontsize=8)
for a in ax:a.grid(alpha=.18)
fig.tight_layout();fig.savefig(run/'地面倾斜与重力估计.png');plt.close(fig)
static=r['scan_summaries']['stationary_raw_plane']['p95_abs']*1000;moving=r['scan_summaries']['moving_raw_plane']['p95_abs']*1000;raw=r['scan_summaries']['moving_truth']['max_abs']*1000;original=r['maps']['localization.pcd'];nav=r['maps']['navigation.pcd'];gr=r['gravity_estimate']
text=f'''# 地面凹凸来源检查

结论：真实仿真地面是平的，原始射线测量没有呈现最终地图里的厘米级地面变形。主要证据指向行走建图期间的姿态/重力方向估计变化，之后被体素化和导航通行检查放大；不能仅归因于缺少人工修图。

## 本轮做了什么

只读检查原始建图运行 d1_full_room_survey_20261109_02。抽样 {r['selected_scans']} 帧，包括 30 帧静止、180 帧运动、20 帧过渡状态。利用保存的逐点时间戳、200 Hz 机身姿态和传感器安装矩阵，离线还原真实地面回波。没有启动仿真或导航，没有编辑 PCD、OctoMap 或图层。

## 结果

| 检查项 | 结果 | 含义 |
|---|---|---|
| 静止扫描地面相对单帧最佳平面的残差 P95 | {static:.3f} mm | 原始静止扫描基本平整 |
| 运动扫描，未做逐点运动补偿的单帧平面残差 P95 | {moving:.1f} mm | 行走扫描确实存在预期的运动畸变，不可把未补偿原始扫描直接拼图 |
| 按每点真实姿态还原后的运动地面回波 | 最大误差 {raw:.4f} mm | 几何/逐点时间信息可正确还原水平地面；这是理想仿真数值精度，不是雷达实测精度 |
| 原始定位 PCD 地面相对真实水平面的高度偏差 P95 | {original['error_against_fixed_physical_floor_m']['p95_abs']*100:.2f} cm | 到保存 PCD 时已存在误差，OctoMap 不是最初来源 |
| 导航 PCD 地面整体倾斜 | {nav['best_plane_tilt_deg']:.3f}° | 很多高度变化来自整体倾斜，不全是局部坑洼 |
| 导航 PCD 去除诊断拟合平面后的残差 P95 | {nav['best_plane_residual_m']['p95_abs']*1000:.2f} mm | 整体倾斜之外仍有局部散布和少量离群点；只做诊断，没有把拟合应用到地图 |
| FAST-LIO 重力估计初末方向变化 | {gr['initial_to_final_direction_change_deg']:.3f}° | 建图过程中竖直方向估计发生变化 |
| 最终估计竖直方向与导航 PCD 地面法向的差异 | {gr['final_estimated_vertical_to_navigation_floor_normal_angle_deg']:.3f}° | 两者高度吻合，支持姿态/重力参考误差的解释 |

不能把“运动扫描未补偿时不平”当成 FAST-LIO 没有运动补偿。最终地图的局部平面残差更小，说明补偿/配准起了作用；本轮发现的是剩余误差及竖直参考变化。

## 为什么又变成白圈和绿色小岛

以地图坐标 (0.15, 0.45) m 附近为例，导航 PCD 的 5 个地面点高度在 −0.32869～−0.29956 m。其中只有 1 个点越过 −0.300 m 的体素分界。当前转换参数为 0.1 m 分辨率、每体素最少 1 点，因此多形成一层占据体素。0.43 m 的机器人半径检查排除周边区域，形成白圈；现有支撑判定又允许小凸起上方的空闲格成为可通行格，留下绿色小岛。

这是“建图估计误差 / 地面高程变化 → 体素离散与单点占据 → 通行判定放大”的组合问题。它不是实际地面突然多了一个 10 cm 障碍物，也不是 Web 随机绘制。

## 已确认与尚未确认

- 已确认：原始射线几何和时间信息能够还原平地；最终 PCD 已存在倾斜与残差；重力估计方向的变化与最终地面法向一致。
- 尚未确认：为什么重力估计会变化，具体是仿真 IMU 的离散合成/时序、估计参数、还是滤波与地图约束共同作用，不能仅凭本轮结果判定某一处代码有 bug。
- 原始记录没有保存逐帧配准后的完整点云和完整后验姿态，不能严格拆分运动补偿、姿态估计、配准和融合各自的贡献。现有 mat_pre 是前验日志，Euler 列还存在旧值问题，本报告仅使用其中有效的重力向量，不用它冒充后验轨迹。
- 初次分析有 1 条柱体角点回波因约 4 微米的包围盒边界差异被误标为地面；第二版使用 10 微米几何容差修正该分类。首版诊断完整保留在 ground_diagnosis_20261116_01，没有通过按观测高度裁掉异常值来美化地面误差。
- 仿真雷达未启用测距噪声，不模拟真实 Livox 扫描花纹和自身遮挡；结论不能作为真机验收。

## 下一步建议

先沿着 IMU 合成/时间戳和 FAST-LIO 重力方向变化这条线定位原因，之后再决定需要修复哪一环。人工导航图层编辑继续保留。多层地图不能全局压平，不能删除所有小障碍，也不能只缩小机器人半径或在网页上涂绿。任何导航预处理都必须保留真实楼板、坡道、楼梯和障碍，并与定位地图保持坐标契约。

## 文件

服务器诊断目录：{run}

report.json 保存逐项统计与源地图 SHA256；scans.json 保存逐帧统计；gravity_series.npz 保存重力方向曲线；两个样本 NPZ 与图片用于检查和复现。分析脚本为 tests/analyze_ground_20261116.py。
'''
(run/'检查报告.md').write_text(text)
checks=dict(localization_map_hash_matches_reference=original['sha256']=='323c21632a8c76eb23f73374e5720916b41d339356182cc4554ae5cbe9fd6b72',navigation_map_hash_matches_reference=nav['sha256']=='5a41314908228206dd5f0a2d7897fa86d8a6b195226d59788887342d2bcce456',raw_floor_reconstruction_max_below_10micrometre=r['scan_summaries']['moving_truth']['max_abs']<1e-5,estimated_vertical_matches_floor_within_0_1deg=gr['final_estimated_vertical_to_navigation_floor_normal_angle_deg']<.1,source_maps_modified=False,simulation_started=False)
(run/'verification.json').write_text(json.dumps(checks,indent=2)+'\n');print(text)
