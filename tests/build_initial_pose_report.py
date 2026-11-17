#!/usr/bin/env python3
import json,csv,collections,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
root=Path('/data0/rl/quadruped_nav/runs/initial_pose_coverage_20261113_01');out=root/'report';out.mkdir(exist_ok=True)
rows=[json.loads(s) for s in (root/'core/results.jsonl').read_text().splitlines()];cases=json.loads((root/'scans/cases.json').read_text());scored=[json.loads(p.read_text()) for p in sorted((root/'full').glob('*/scored.json'))];names=list(dict.fromkeys(r['scenario'] for r in rows));summary={}
for name in names:
 group=[r for r in rows if r['scenario']==name];acc=[r for r in group if r['accepted']];summary[name]=dict(total=len(group),correct=sum(r['outcome']=='correct_accept' for r in group),accepted_over_limit=sum(r['outcome']=='false_accept' for r in group),rejected=sum(not r['accepted'] for r in group),gross_false_accept=sum(r['accepted'] and (r['position_error_m']>.5 or r['rotation_error_deg']>30) for r in group),reasons=dict(collections.Counter(r['reason'] for r in group)))
(out/'summary.json').write_text(json.dumps(dict(core=summary,full=scored),indent=2)+'\n')
keys=['case','location','xy','yaw_deg','scenario','outcome','reason','fitness','rmse_m','position_error_m','rotation_error_deg','yaw_error_deg','correction_shift_m','correction_rotation_rad']
with (out/'all_core_trials.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore');w.writeheader();w.writerows(rows)
fig,axes=plt.subplots(1,7,figsize=(18,10),sharey=True,layout='constrained');cmap=ListedColormap(['#b5bac2','#39aa6b','#efae42','#d94848'])
for ax,name in zip(axes,names):
 matrix=np.zeros((19,8),dtype=int)
 for r in rows:
  if r['scenario']!=name:continue
  color=0 if not r['accepted'] else 1 if r['outcome']=='correct_accept' else 3 if r['position_error_m']>.5 or r['rotation_error_deg']>30 else 2
  matrix[r['location'],r['yaw_deg']//45]=color
 ax.imshow(matrix,cmap=cmap,vmin=0,vmax=3,aspect='auto');ax.set_xticks(range(8),range(0,360,45),rotation=90);ax.set_title(name.replace('_','\n'),fontsize=10);ax.set_xlabel('Robot yaw (deg)')
axes[0].set_yticks(range(19),[str(next(c['map_xy'] for c in cases if c['location']==i)) for i in range(19)]);axes[0].set_ylabel('Actual map XY (m)')
from matplotlib.patches import Patch
fig.legend(handles=[Patch(color=cmap(i),label=l) for i,l in enumerate(['Rejected','Accepted within 0.18 m / 5 deg','Accepted over test limits','Accepted error >0.5 m or >30 deg'])],loc='lower center',bbox_to_anchor=(.5,-.025),ncol=4)
fig.suptitle('Initial localization coverage: 19 locations x 8 headings x 7 seeds\nStatic Isaac raycast scans + unchanged production ICP; not full runtime / hardware validation',fontsize=14);fig.savefig(out/'coverage.png',dpi=150,bbox_inches='tight');plt.close(fig)
lines=['# 初始定位覆盖测试','', '结论：不能保证任意初值定位成功。当前是有初值的局部 ICP（迭代最近点）配准；较大误差下存在误定位被接受。','', '## 测试范围','', '- 使用现有完整房间地图，19 个位置、8 个朝向（每隔 45°），每次单独初始化，7 组初值，共 1,064 次静态核心配准。','- 准确初值组也只输入平面位姿，roll/pitch（横滚/俯仰）和 z 设为 0，并未直接提供完整三维真值。','- 独立完整链路抽样：Isaac 物理传感器 → FAST-LIO → 单次 `/initialpose` → 当前定位器。初值带预设误差；仿真真值仅离线评分，不反馈给算法。','- 保留生产配准门限、地图、算法；完整链路采用上一版成功演示的定位配置（静止速度阈值 0.12 m/s，跟踪从精配准层开始），不是全部默认参数。','- 评分标准：位置误差 ≤ 0.18 m，整体旋转误差 ≤ 5°。这是本次工程测试阈值，不是雷达或真机精度承诺。超限不全是严重误定位，另统计位置 > 0.5 m 或旋转 > 30° 的明显错误。','', '## 静态核心结果','', '| 初值 | 正确接受 | 接受但超限 | 拒绝 | 其中明显错误仍接受 |','|---|---:|---:|---:|---:|']
labels={'accurate':'位置/朝向准确','coarse_10deg':'平移 +0.35/-0.20 m，偏航 +10°','coarse_20deg':'平移 -0.50/+0.40 m，偏航 -20°','wrong_yaw_90deg':'偏航错误 +90°','wrong_yaw_180deg':'偏航错误 +180°','wrong_position_2m':'位置错误 +2 m','fixed_origin':'所有摆位均给原点/0° 初值'}
for name in names:
 s=summary[name];lines.append(f"| {labels[name]} | {s['correct']}/{s['total']} | {s['accepted_over_limit']} | {s['rejected']} | {s['gross_false_accept']} |")
lines+=['','## 完整运行链路抽样','', '| 实际初始 XY / 朝向 | 输入误差 XY / 朝向 | 首次位置误差 | 首次偏航误差 | 6 s 观察期位置 P95 | 判定 |','|---|---|---:|---:|---:|---|']
for r in scored:
 first=r.get('first_pose_errors',{});pos=r.get('position_error_m',{});fmt=lambda v,u:f'{v:.3f}{u}' if v is not None else '无结果'
 verdict='通过' if r.get('within_test_limits') and r.get('first_pose_within_limits') else '已接受但误差超限' if r.get('truth_comparison_samples',0) else '未得到定位结果'
 lines.append(f"| {r['requested_spawn']} | {r['seed_error']} | {fmt(first.get('position_error_m'),' m')} | {fmt(first.get('yaw_error_deg'),'°')} | {fmt(pos.get('p95'),' m')} | {verdict} |")
lines+=['','每组 `/initialpose` 只发送 1 次，不自动重试。完整链路中“进入 tracking”并不等于误差合格；对每条姿态消息使用原始消息时间戳，与物理状态插值比较。传感器时钟相对物理时钟的固定 2 s 偏移来自现有仿真代码。','', '## 原因与边界','', '- 雷达自身旋转朝向任意，与初值朝向错误是两件事。实际朝向为 180°、同时给出约 180° 初值，可以正常进行局部配准。','- ICP 从给定初值附近找局部最优，不执行整张地图的全位置/全角度搜索。长直墙、地面、天花板和重复几何可能让错误位置仍有很低的残差。','- 当前 fitness（匹配比例）、RMSE（均方根误差）与修正跳变门限不能验证解的唯一性。跳变门限限制相对于初值的修正量；若错误初值附近存在局部最优，修正很小也可能被接受。','- 静态覆盖使用 6,000 射线/帧、10 Hz、0.1–20 m 的现有几何代理。未模拟厂家扫描花纹、真实噪声、机器人自身遮挡，不代表真实 Mid-360S 验收。','- 有限房间、离散摆位与单次扫描不能证明“任意点位都成功”；未覆盖动态障碍、室外、楼梯、严重遮挡、雷达或 IMU 实际误差。','- 地图由 LIO 建立，相对物理几何可能存在自身误差；评分没有拟合额外对齐变换来美化结果。','', '## 建议下一步','', '先处理错误初值被接受的问题，再做 Web 上位机。若仍采用当前局部 ICP，应明确要求合理初值并增加初始化可信度验证；若希望位置/朝向未知也能恢复，需要独立的全局候选搜索与歧义判定。具体算法改动另行设计，本轮没有修改生产算法。','', '## 证据','', f'- 服务器运行目录：`{root}`','- `core/results.jsonl` 保存全部逐次结果与候选变换；`scans/` 保存原始扫描与静态真值；`full/case*/` 保存完整链路日志、初值事件、原始姿态时间戳、物理状态和误差。','- `report/all_core_trials.csv` 提供便于筛选的结果；`core/contract.json` 保存原配置、生产配准代码哈希和地图哈希。','']
(out/'测试报告.md').write_text('\n'.join(lines));print(json.dumps(summary,indent=2));print('full_cases',len(scored))
