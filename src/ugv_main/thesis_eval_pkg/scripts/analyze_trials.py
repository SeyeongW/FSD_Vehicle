#!/usr/bin/env python3
"""
analyze_trials.py — 졸업논문 단일/다중 LiDAR 비교 결과 집계

[지표 정의]
  탐지 성공률    : 융합 클라우드에서 GT 반경 0.5m 이내 포인트가 MIN_HITS(=DBSCAN min_samples 3)개 이상인 프레임 비율
  추적 유지율    : 최근 0.5s 이내 /bird_target 추정이 있고 GT와 1m 이내인 프레임 비율
  위치 추정 오차 : 추정이 있는 프레임의 |추정 - GT| 평균 / RMSE (m)
  처리 시간      : 융합 노드 1회 처리 시간(ms, wall)
  새 포인트 수   : 프레임당 GT 반경 0.5m 이내 융합 포인트 수 평균
  최장 추적 중단 : 추적 실패가 연속된 최장 시간(s)
  정상 구간 = bird_t ∈ [WARMUP, FAULT_DELAY),  고장 구간 = bird_t ≥ FAULT_DELAY

[사용]  python3 analyze_trials.py [로그폴더]  (기본 ~/ros2_ws/ugv_ws/logs/thesis)
[출력]  <로그폴더>/results/summary.md, fig_*.png
"""

import glob
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

MIN_HITS = 3
WARMUP = 5.0
FAULT_DELAY = 60.0
DIST_BINS = np.arange(0, 26, 2.0)
SOURCES = ['lidar1', 'lidar2', 'mid360']
BLIND_ELEV_DEG = 55.0   # Mid-360 수직 시야 상한 근처 — 이 이상은 상부 탐지 불가 영역
NAME_RE = re.compile(r'thesis_(single|multi)_(none|mid360|lidar1|lidar2)_s(-?\d+)_')


def load(log_dir):
    frames = []
    for path in sorted(glob.glob(os.path.join(log_dir, 'thesis_*.csv'))):
        m = NAME_RE.search(os.path.basename(path))
        if not m:
            continue
        try:
            df = pd.read_csv(path)
        except pd.errors.EmptyDataError:
            continue
        if df.empty:
            continue
        df['mode'], df['fault'], df['seed'] = m.group(1), m.group(2), int(m.group(3))
        df['trial'] = os.path.basename(path)
        df['detected'] = df['total_hits'] >= MIN_HITS
        frames.append(df)
    if not frames:
        sys.exit(f'CSV 없음: {log_dir}')
    return pd.concat(frames, ignore_index=True)


def metrics(df):
    err = df['pos_err'].dropna()
    return {
        'frames': len(df),
        'detect_rate_%': 100 * df['detected'].mean(),
        'track_cont_%': 100 * df['tracked'].mean(),
        'pos_err_mean_m': err.mean(),
        'pos_err_rmse_m': np.sqrt((err ** 2).mean()),
        'proc_ms': df['proc_ms'].mean(),
        'bird_hits': df['total_hits'].mean(),
        'max_track_gap_s': _max_gap(df['tracked'].to_numpy()),
    }


def _max_gap(tracked, dt=0.1):
    """추적 실패(tracked=0)가 연속된 최장 구간(s). 로거 주기 10Hz."""
    run = best = 0
    for x in tracked:
        run = run + 1 if x == 0 else 0
        best = max(best, run)
    return best * dt


def _agg(sub):
    """트라이얼(seed)별 지표를 낸 뒤 평균 ± 표준편차."""
    per = pd.DataFrame([metrics(g) for _, g in sub.groupby('trial')])
    row = {'trials': len(per), 'frames': int(per['frames'].sum())}
    for c in per.columns.drop('frames'):
        sd = per[c].std(ddof=1) if len(per) > 1 else float('nan')
        row[c] = f'{per[c].mean():.2f} ± {sd:.2f}' if len(per) > 1 else f'{per[c].mean():.2f}'
    return row


def summary_table(all_df):
    rows = []
    normal = all_df[(all_df['bird_t'] >= WARMUP) & (all_df['bird_t'] < FAULT_DELAY)]
    for mode in ['single', 'multi']:
        sub = normal[normal['mode'] == mode]
        if len(sub):
            rows.append({'condition': f'{mode} / 정상', **_agg(sub)})
    fault = all_df[all_df['bird_t'] >= FAULT_DELAY]
    for (mode, f), sub in fault.groupby(['mode', 'fault']):
        if f != 'none':
            rows.append({'condition': f'{mode} / {f} 고장', **_agg(sub)})
    return pd.DataFrame(rows), normal


def elevation_table(normal):
    """이유 3: 온보드 mid360 기준 새 앙각별 포인트/탐지 (mid360 높이는 TF 거리와 GT 고도로 역산하지 않고 0.3m 근사)."""
    m = normal[(normal['mid360_alive'] == 1)].copy()
    m['elev'] = _elevation(m)
    rows = []
    for lo, hi in [(0, 40), (40, 55), (55, 90)]:
        s = m[(m['elev'] >= lo) & (m['elev'] < hi)]
        rows.append({'elevation': f'{lo}-{hi}°', 'frame_%': 100 * len(s) / max(len(m), 1),
                     'mean_dist_m': s['mid360_dist'].mean(), 'mean_hits': s['mid360_hits'].mean(),
                     'mid360_detect_%': 100 * (s['mid360_hits'] >= MIN_HITS).mean()})
    return pd.DataFrame(rows)


def fig_detect_vs_distance(normal, out):
    """이유 2: 로봇(온보드 센서)-새 거리별 탐지율 — 단일은 거리↑ 탐지↓, 다중은 고정 센서가 보완."""
    fig, ax = plt.subplots(figsize=(6, 4))
    centers = (DIST_BINS[:-1] + DIST_BINS[1:]) / 2
    for mode, marker in [('single', 'o'), ('multi', 's')]:
        sub = normal[(normal['mode'] == mode)].dropna(subset=['mid360_dist'])
        b = pd.cut(sub['mid360_dist'], DIST_BINS)
        g = sub.groupby(b, observed=False)['detected']
        rate, cnt = 100 * g.mean().values, g.size().values
        ok = cnt >= 10
        ax.plot(centers[ok], rate[ok], marker=marker, label=f'{mode} LiDAR')
    ax.set_xlabel('Distance between UGV (onboard LiDAR) and bird [m]')
    ax.set_ylabel('Detection rate [%]')
    ax.set_ylim(0, 105)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_detect_vs_distance.png'), dpi=200)
    plt.close(fig)


def _elevation(df):
    """온보드 mid360 기준 새의 앙각(deg). mid360 높이는 0.3m로 근사."""
    horiz = np.hypot(df['gt_x'] - df['robot_x'], df['gt_y'] - df['robot_y'])
    return np.degrees(np.arctan2(df['gt_z'] - 0.3, horiz))


# 센서별 색(검증된 기본 팔레트 slot 1~3, 고정 순서) + 마커(흑백 인쇄·색각 대비 2차 구분)
SENSOR_STYLE = {
    'lidar1': ('Fixed LiDAR 1', '#2a78d6', 'o'),
    'lidar2': ('Fixed LiDAR 2', '#eb6834', 's'),
    'mid360': ('Onboard Mid-360 (elev. < 55°)', '#1baf7a', '^'),
}


def fig_hits_vs_distance(normal, out):
    """센서별 새 포인트 수 vs 자기 센서-새 거리. 1 m 구간 평균(마커) + N = k/d^2 최소제곱 피팅(곡선).
    온보드 mid360은 상부 탐지 불가 영역(앙각 55° 이상) 프레임을 제외 — 거리 효과만 보기 위함."""
    from matplotlib.lines import Line2D
    fig, ax = plt.subplots(figsize=(6, 4))
    multi = normal[normal['mode'] == 'multi']
    bins = np.arange(1.0, 22.0, 1.0)
    fit_lines, handles = [], []
    for s in SENSOR_STYLE:
        col_d, col_h, col_a = f'{s}_dist', f'{s}_hits', f'{s}_alive'
        if col_d not in multi:
            continue
        sub = multi[(multi[col_a] == 1)].dropna(subset=[col_d])
        sub = sub[sub[col_d] > 0.5]
        if s == 'mid360':
            sub = sub[_elevation(sub) < BLIND_ELEV_DEG]
        if sub.empty:
            continue
        label, color, marker = SENSOR_STYLE[s]
        g = sub.groupby(pd.cut(sub[col_d], bins), observed=False)[col_h]
        mean, cnt = g.mean().to_numpy(), g.size().to_numpy()
        centers = (bins[:-1] + bins[1:]) / 2
        ok = (cnt >= 20) & (mean > 0)
        ax.plot(centers[ok], mean[ok], ls='none', marker=marker, ms=6, color=color,
                markeredgecolor='white', markeredgewidth=0.8, zorder=3)
        k = np.sum(sub[col_h] / sub[col_d] ** 2) / np.sum(1 / sub[col_d] ** 4)   # 최소제곱 N = k/d^2
        d = np.linspace(1.5, 21.0, 200)
        ax.plot(d, k / d ** 2, lw=2, color=color, zorder=2)
        fit_lines.append((s, k))
        handles.append(Line2D([], [], color=color, marker=marker, lw=2, ms=6,
                              markeredgecolor='white', markeredgewidth=0.8,
                              label=f'{label}: k = {k:.0f}, N = 3 at {np.sqrt(k / MIN_HITS):.1f} m'))
    ax.axhline(MIN_HITS, color='#555555', ls='--', lw=1, zorder=1)
    ax.text(21.0, MIN_HITS * 1.12, f'min_samples = {MIN_HITS}', ha='right', va='bottom', fontsize=8, color='#555555')
    ax.set_xlabel('Sensor-to-bird distance [m]')
    ax.set_ylabel('Points on bird per frame')
    ax.set_yscale('log')
    ax.set_ylim(0.5, 300)
    ax.set_xlim(0, 21.5)
    ax.grid(alpha=0.25, lw=0.6, which='major')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.legend(handles=handles, fontsize=7.5, frameon=False, loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_hits_vs_distance.png'), dpi=300)
    plt.close(fig)
    return fit_lines


def fig_fault_timeline(all_df, out):
    """이유 1: 고장 전후 탐지율 시계열 (1s 구간, seed 평균)."""
    fig, ax = plt.subplots(figsize=(7, 4))
    for (mode, f), sub in all_df[all_df['fault'] != 'none'].groupby(['mode', 'fault']):
        t = np.floor(sub['bird_t'])
        series = 100 * sub.groupby(t)['detected'].mean()
        series = series.rolling(3, center=True, min_periods=1).mean()
        ax.plot(series.index.to_numpy(), series.to_numpy(), label=f'{mode} / {f} fault')
    ax.axvline(FAULT_DELAY, color='r', ls='--', lw=1, label='fault injected')
    ax.set_xlabel('Time since bird appeared [s]')
    ax.set_ylabel('Detection rate [%] (1 s bins)')
    ax.set_ylim(-3, 105)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'fig_fault_timeline.png'), dpi=200)
    plt.close(fig)


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/ros2_ws/ugv_ws/logs/thesis')
    out = os.path.join(log_dir, 'results')
    os.makedirs(out, exist_ok=True)

    all_df = load(log_dir)
    table, normal = summary_table(all_df)
    fig_detect_vs_distance(normal, out)
    fits = fig_hits_vs_distance(normal, out)
    fig_fault_timeline(all_df, out)

    trials = all_df.groupby(['mode', 'fault'])['trial'].nunique()
    with open(os.path.join(out, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write('# 단일/다중 LiDAR 비교 결과\n\n')
        f.write('## 트라이얼 수\n\n' + trials.to_frame('n').to_markdown() + '\n\n')
        f.write('## 성능 비교 (트라이얼별 지표의 평균 ± 표준편차)\n\n' + table.to_markdown(index=False) + '\n\n')
        f.write('## mid360 앙각별 탐지 (정상 구간, single+multi)\n\n'
                + elevation_table(normal).to_markdown(index=False, floatfmt='.1f') + '\n\n')
        f.write('## 포인트 수 거리 피팅 N = k/d² (mid360은 앙각 55° 이상 제외)\n\n')
        for s, k in fits:
            f.write(f'- {s}: k = {k:.1f}  → N=3이 되는 거리 ≈ {np.sqrt(k / MIN_HITS):.1f} m\n')
    print(open(os.path.join(out, 'summary.md'), encoding='utf-8').read())
    print(f'그림 저장: {out}')


if __name__ == '__main__':
    main()
