#!/usr/bin/env python3
"""
Fig. 4 — 3D LiDAR 데이터 처리 흐름도 (두 갈래 분기)
실행: python3 fig4_pipeline.py
출력: fig4_pipeline.png (300 dpi)
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib import font_manager

font_manager.fontManager.addfont(
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
)
plt.rcParams['font.family'] = 'Noto Sans CJK JP'

fig, ax = plt.subplots(figsize=(14, 16))
ax.set_xlim(0, 14)
ax.set_ylim(0, 16)
ax.axis('off')

# ── 색상 ──────────────────────────────────────────────────────────────
C_INPUT  = '#2C3E50'   # 다크 네이비  — 입력
C_NAV    = '#1A5276'   # 딥 블루      — 자율주행 경로
C_BIRD   = '#1E8449'   # 딥 그린      — 조류 탐지 경로
C_OUT_N  = '#5DADE2'   # 라이트 블루  — 자율주행 출력
C_OUT_B  = '#58D68D'   # 라이트 그린  — 조류 탐지 출력
C_YOLO   = '#7D3C98'   # 퍼플         — YOLO 보조
C_SPLIT  = '#E67E22'   # 오렌지       — 분기점
TEXT_W   = 'white'
TEXT_B   = 'white'

def box(ax, x, y, w, h, text, fc, tc=TEXT_W, fs=9.5, bold=False):
    rect = FancyBboxPatch(
        (x - w/2, y - h/2), w, h,
        boxstyle="round,pad=0.08",
        facecolor=fc, edgecolor='white', linewidth=1.2, zorder=3
    )
    ax.add_patch(rect)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, ha='center', va='center',
            fontsize=fs, color=tc, weight=weight, zorder=4,
            multialignment='center')

def arrow(ax, x1, y1, x2, y2, color='#BDC3C7', lw=1.8):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color,
                                lw=lw, connectionstyle='arc3,rad=0.0'),
                zorder=2)

def arrow_curved(ax, x1, y1, x2, y2, color='#BDC3C7', lw=1.8, rad=0.15):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color,
                                lw=lw, connectionstyle=f'arc3,rad={rad}'),
                zorder=2)

# ══════════════════════════════════════════════════════════════════════
# 1. 공통 입력
# ══════════════════════════════════════════════════════════════════════
box(ax, 7, 15.2, 5.5, 0.7,
    'Livox MID-360  PointCloud2', C_INPUT, fs=10.5, bold=True)

# 분기 선 (수직 → 좌우)
ax.plot([7, 7],   [14.85, 14.3], color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([3.5, 7], [14.3,  14.3], color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([7, 10.5],[14.3,  14.3], color='#BDC3C7', lw=1.8, zorder=2)
arrow(ax, 3.5, 14.3, 3.5, 13.75)
arrow(ax, 10.5, 14.3, 10.5, 13.75)

# 분기 레이블
ax.text(2.2, 14.55, '경로 ①  자율주행', fontsize=9, color=C_NAV,
        weight='bold', zorder=4)
ax.text(9.0, 14.55, '경로 ②  조류 탐지·추적', fontsize=9, color=C_BIRD,
        weight='bold', zorder=4)

# ══════════════════════════════════════════════════════════════════════
# 2. 경로 ① — 자율주행 (왼쪽, x=3.5)
# ══════════════════════════════════════════════════════════════════════
nav_steps = [
    (13.45, 'Range Filter\n수평 거리  0.2 m ~ 20.0 m'),
    (12.45, 'Height Filter\n높이  0.10 m ~ 1.50 m\n(지면 및 상단 초과 제거)'),
    (11.25, 'pointcloud_to_laserscan\n노드'),
    (10.25, '/scan\n(2D LaserScan,  base_link)'),
]
for i, (y, txt) in enumerate(nav_steps):
    h = 0.75 if '\n' not in txt else (0.85 if txt.count('\n') == 1 else 0.95)
    box(ax, 3.5, y, 4.6, h, txt, C_NAV, fs=9)
    if i < len(nav_steps) - 1:
        arrow(ax, 3.5, y - h/2, 3.5, nav_steps[i+1][0] + 0.48)

# /scan 이후 분기
y_scan_bot = 10.25 - 0.43
ax.plot([3.5, 3.5], [y_scan_bot, 9.55], color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([1.8, 3.5], [9.55, 9.55],       color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([3.5, 5.2], [9.55, 9.55],       color='#BDC3C7', lw=1.8, zorder=2)
arrow(ax, 1.8, 9.55, 1.8, 9.05)
arrow(ax, 5.2, 9.55, 5.2, 9.05)

box(ax, 1.8, 8.65, 2.6, 0.75, 'Cartographer\n(사전 맵 구축)', C_OUT_N, fs=9)
box(ax, 5.2, 8.65, 2.6, 0.75, 'Nav2 Stack\nAMCL + 경로 수립', C_OUT_N, fs=9)

# ══════════════════════════════════════════════════════════════════════
# 3. 경로 ② — 조류 탐지·추적 (오른쪽, x=10.5)
# ══════════════════════════════════════════════════════════════════════
bird_steps = [
    (13.45, 'ROI Filter\n수평 거리 0.3~15.0 m,  z > 2.1 m'),
    (12.45, 'Self-Point 제거\n(로봇 본체 영역 제거)'),
    (11.45, 'DBSCAN 클러스터링\neps = 0.6 m,  min_samples = 3'),
    (10.45, '클러스터 유효성 검사\n크기 < 2.0×2.0×1.5 m,  중심 z > 2.0 m'),
    (9.40,  'TF 변환\nLiDAR 프레임 → odom 프레임'),
    (8.40,  '칼만 필터\n[px, py, pz, vx, vy, vz]  |  lookahead 0.15 s'),
    (7.35,  '로컬 프레임 변환\nodom → 로봇 로컬  (cx, cy, cz)'),
]
for i, (y, txt) in enumerate(bird_steps):
    h = 0.75 if txt.count('\n') == 1 else 0.85
    box(ax, 10.5, y, 5.0, h, txt, C_BIRD, fs=9)
    if i < len(bird_steps) - 1:
        next_h = 0.75 if bird_steps[i+1][1].count('\n') == 1 else 0.85
        arrow(ax, 10.5, y - h/2, 10.5, bird_steps[i+1][0] + next_h/2)

# 로컬 프레임 변환 이후 분기
y_local_bot = 7.35 - 0.43
ax.plot([10.5, 10.5], [y_local_bot, 6.5], color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([8.8, 10.5],  [6.5, 6.5],         color='#BDC3C7', lw=1.8, zorder=2)
ax.plot([10.5, 12.2], [6.5, 6.5],         color='#BDC3C7', lw=1.8, zorder=2)
arrow(ax, 8.8, 6.5, 8.8, 6.0)
arrow(ax, 12.2, 6.5, 12.2, 6.0)

box(ax, 8.8, 5.65, 3.0, 0.65,
    'Pure Pursuit + P제어\n/cmd_vel', C_OUT_B, fs=9)
box(ax, 12.2, 5.65, 3.0, 0.65,
    '카메라 틸트 제어\n/set_joint_trajectory', C_OUT_B, fs=9)

# ══════════════════════════════════════════════════════════════════════
# 4. YOLO 시각 보조 (폴백)
# ══════════════════════════════════════════════════════════════════════
box(ax, 10.5, 4.3, 5.0, 0.75,
    'YOLOv8  (bird_yolo_node)\n짐벌 카메라 → /bird_visual_bearing', C_YOLO, fs=9)

# YOLO 연결: 칼만필터 유실 시 → YOLO → cmd_vel
ax.annotate('', xy=(10.5, 4.68), xytext=(10.5, 6.93),
            arrowprops=dict(arrowstyle='<-', color='#A569BD',
                            lw=1.4, linestyle='dashed',
                            connectionstyle='arc3,rad=0.35'),
            zorder=2)
ax.text(12.6, 5.8, 'LiDAR 트랙\n유실 시 폴백', fontsize=7.5,
        color='#A569BD', style='italic', ha='center')

arrow(ax, 10.5, 3.92, 8.8, 6.0 + 0.33)

# ══════════════════════════════════════════════════════════════════════
# 5. 제목 및 범례
# ══════════════════════════════════════════════════════════════════════
ax.set_facecolor('#1C2833')
fig.patch.set_facecolor('#1C2833')

legend_items = [
    mpatches.Patch(color=C_NAV,   label='자율주행 경로'),
    mpatches.Patch(color=C_BIRD,  label='조류 탐지·추적 경로'),
    mpatches.Patch(color=C_OUT_N, label='자율주행 출력'),
    mpatches.Patch(color=C_OUT_B, label='조류 추적 출력'),
    mpatches.Patch(color=C_YOLO,  label='YOLO 시각 보조 (폴백)'),
]
ax.legend(handles=legend_items, loc='lower left',
          fontsize=8.5, framealpha=0.25,
          facecolor='#2C3E50', edgecolor='gray', labelcolor='white')

ax.text(7, 0.25, 'Fig. 4.  3D LiDAR Data Processing Flow',
        ha='center', va='bottom', fontsize=11,
        color='white', weight='bold')

plt.tight_layout(pad=0.3)
plt.savefig('fig4_pipeline.png', dpi=300, bbox_inches='tight',
            facecolor=fig.get_facecolor())
print('저장 완료: fig4_pipeline.png')
