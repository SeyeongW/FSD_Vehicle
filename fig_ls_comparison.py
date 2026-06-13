#!/usr/bin/env python3
"""
실제 LiDAR 포인트 클라우드 데이터로 RANSAC vs RANSAC+LS 비교 그림 생성.
먼저 시뮬레이션을 실행해 /tmp/plane_fit_data.npz 를 생성한 뒤 실행하세요.
  python3 fig_ls_comparison.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math

DATA_PATH = '/tmp/plane_fit_data.npz'

# ── 데이터 로드 ────────────────────────────────────────────────────────────────
try:
    data = np.load(DATA_PATH)
except FileNotFoundError:
    print(f'[오류] {DATA_PATH} 없음.')
    print('시뮬레이션을 실행하고 5프레임 처리될 때까지 기다려 주세요.')
    sys.exit(1)

points      = data['points']           # Nx3  (x, y, z)
inliers_r   = data['inliers_ransac']   # 인덱스 배열
inliers_l   = data['inliers_ls']
plane_r     = data['plane_ransac']     # [a, b, c, d]
plane_l     = data['plane_ls']

print(f'로드 완료: {len(points)} points, '
      f'RANSAC inliers={len(inliers_r)}, LS inliers={len(inliers_l)}')

# ── 인라이어 / 아웃라이어 분리 ─────────────────────────────────────────────────
outlier_mask = np.ones(len(points), dtype=bool)
outlier_mask[inliers_l] = False

in_pts  = points[inliers_l]
out_pts = points[outlier_mask]

# ── 2D 투영: x-z 단면 (수평 거리 vs 높이) ─────────────────────────────────────
# 시각화를 위해 포인트 수 제한 (최대 3000개)
rng = np.random.default_rng(0)
def sample(pts, n=3000):
    if len(pts) <= n:
        return pts
    return pts[rng.choice(len(pts), n, replace=False)]

in_s  = sample(in_pts)
out_s = sample(out_pts)

# x 범위
x_all = points[:, 0]
xp    = np.linspace(-10, 10, 400)

def plane_line_xz(plane, xp, y=0.0):
    """ax + by + cz + d = 0  →  z = (-a*x - b*y - d) / c"""
    a, b, c, d = plane
    if abs(c) < 1e-9:
        return None
    return (-a * xp - b * y - d) / c

z_line_r = plane_line_xz(plane_r, xp)
z_line_l = plane_line_xz(plane_l, xp)

# 평면의 각도 오차 계산
def angle_err(plane):
    n = plane[:3] / (np.linalg.norm(plane[:3]) + 1e-12)
    dot = abs(float(np.dot(n, [0, 0, 1])))
    return math.degrees(math.acos(min(1.0, dot)))

theta_r = angle_err(plane_r)
theta_l = angle_err(plane_l)

# RMSE (인라이어 기준)
def rmse(pts, plane):
    a, b, c, d = plane
    dists = np.abs(pts @ plane[:3] + d)
    return float(np.sqrt(np.mean(dists**2)))

rmse_r = rmse(in_pts, plane_r) * 100   # cm
rmse_l = rmse(in_pts, plane_l) * 100

# ── 실제 측정 통계 (30프레임 평균) ────────────────────────────────────────────
STATS = dict(
    ransac=dict(ir='0.505±0.006', rmse='1.09±0.25 cm', angle='0.095±0.016 deg'),
    ls    =dict(ir='0.503±0.006', rmse='0.65±0.05 cm', angle='0.024±0.009 deg'),
)

# ── 그림 ────────────────────────────────────────────────────────────────────────
COLORS = dict(inlier='#2ECC40', outlier='#FF4136',
              sample='royalblue', ransac='royalblue',
              ls='#FF6B35', truth='#555555')

fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharey=True)
fig.suptitle(
    'RANSAC vs RANSAC + Least Squares  —  Real LiDAR Point Cloud (Gazebo)',
    fontsize=13, fontweight='bold', y=1.01)

titles = [
    f'① Custom RANSAC\n(plane from 3 sampled points)',
    f'② RANSAC + Least Squares\n(plane refined with ALL inliers via Normal Equation)',
]

for i, ax in enumerate(axes):
    # 정답 바닥
    ax.axhline(0, color=COLORS['truth'], linestyle='--', linewidth=1.4,
               alpha=0.6, zorder=1, label='True floor  z = 0')

    # 아웃라이어 (x-z 투영) — x 범위 안에 있는 것만
    out_view = out_s[(out_s[:, 0] >= -10) & (out_s[:, 0] <= 10)]
    ax.scatter(out_view[:, 0], out_view[:, 2], color=COLORS['outlier'],
               s=8, alpha=0.35, zorder=2, label=f'Outliers  ({len(out_pts)} pts)')

    # 인라이어 (x-z 투영) — 더 크고 진하게
    in_view = in_s[(in_s[:, 0] >= -10) & (in_s[:, 0] <= 10)]
    ax.scatter(in_view[:, 0], in_view[:, 2], color=COLORS['inlier'],
               s=14, alpha=0.75, zorder=4, label=f'Inliers  ({len(in_pts)} pts)')

    # RANSAC 평면선
    if z_line_r is not None:
        lbl_r = f'RANSAC plane  (this frame: angle={theta_r:.3f}deg, RMSE={rmse_r:.2f}cm)'
        ax.plot(xp, z_line_r, color=COLORS['ransac'], linewidth=2.5, zorder=5,
                label=lbl_r, alpha=(1.0 if i == 0 else 0.35))

    if i == 1:
        # LS 평면선
        if z_line_l is not None:
            lbl_l = f'RANSAC+LS plane  (this frame: angle={theta_l:.3f}deg, RMSE={rmse_l:.2f}cm)'
            ax.plot(xp, z_line_l, color=COLORS['ls'], linewidth=2.8, zorder=6, label=lbl_l)

        # 잔차 화살표 (인라이어 샘플 일부)
        arrow_pts = in_s[::max(1, len(in_s)//20)]   # 최대 20개
        a, b, c, d = plane_l
        for p in arrow_pts:
            z_pred = (-a * p[0] - b * p[1] - d) / c
            ax.annotate('', xy=(p[0], z_pred), xytext=(p[0], p[2]),
                        arrowprops=dict(arrowstyle='->', color=COLORS['ls'],
                                        lw=1.0, alpha=0.5))

        # 정보 박스: 30프레임 통계
        s = STATS['ls']
        rmse_imp = (1 - 0.0065 / 0.0109) * 100
        ang_imp  = (1 - 0.024  / 0.095 ) * 100
        info = (f'[30-frame statistics]\n'
                f'Normal Equation  (AᵀA)c = Aᵀz\n'
                f'─────────────────────────────\n'
                f'IR    : {s["ir"]}\n'
                f'RMSE  : {s["rmse"]}  (down {rmse_imp:.0f}%)\n'
                f'Angle : {s["angle"]}  (down {ang_imp:.0f}%)')
        ax.text(0.98, 0.97, info, transform=ax.transAxes,
                ha='right', va='top', fontsize=8.5,
                bbox=dict(boxstyle='round,pad=0.5', fc='#FFF0E8',
                          ec=COLORS['ls'], alpha=0.95))
    else:
        s = STATS['ransac']
        info = (f'[30-frame statistics]\n'
                f'3-point sample only\n'
                f'─────────────────────────────\n'
                f'IR    : {s["ir"]}\n'
                f'RMSE  : {s["rmse"]}\n'
                f'Angle : {s["angle"]}')
        ax.text(0.98, 0.97, info, transform=ax.transAxes,
                ha='right', va='top', fontsize=8.5,
                bbox=dict(boxstyle='round,pad=0.5', fc='#E8F0FF',
                          ec=COLORS['ransac'], alpha=0.95))

    ax.set_xlim(-10, 10)
    ax.set_ylim(-0.3, 2.8)
    ax.set_xlabel('Horizontal distance  x  (m)', fontsize=11)
    ax.set_ylabel('Height  z  (m)', fontsize=11)
    ax.set_title(titles[i], fontsize=11.5, pad=8)
    ax.legend(loc='upper left', fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.2)

fig.text(0.5, -0.03,
         'z = a₀ + a₁x + a₂y   →   Sᵣ = Σ(zᵢ−a₀−a₁xᵢ−a₂yᵢ)² minimized'
         '   →   (AᵀA)c = Aᵀz   →   np.linalg.solve',
         ha='center', fontsize=10.5, style='italic', color='#333333')

plt.tight_layout()
out = '/home/seo/ros2_ws/ugv_ws/fig_ls_comparison.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f'저장 완료: {out}')
