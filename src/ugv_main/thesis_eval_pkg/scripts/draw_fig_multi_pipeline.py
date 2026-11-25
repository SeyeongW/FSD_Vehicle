#!/usr/bin/env python3
"""
draw_fig_multi_pipeline.py — 졸업논문 Fig. 6 (다중 LiDAR 처리 파이프라인)

기존 초안 Fig. 4(연한 파랑 + 이중 테두리 상자, 점선 둥근 입출력, 굵은 세리프)와 같은 스타일로,
LiDAR 3개 입력과 고장 허용 융합 블록을 추가해 다시 그린다.

[사용] python3 draw_fig_multi_pipeline.py [출력경로.png]
"""

import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

NAVY = '#1f2f5a'
FILL = '#dce8f6'
INK = '#111111'
FONT = dict(family='DejaVu Serif', weight='bold', color=INK)


def terminal(ax, x, y, w, h, text, fs=9):
    """점선 테두리의 둥근(stadium) 입출력 상자."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle=f'round,pad=0,rounding_size={h / 2}',
                                fc=FILL, ec=NAVY, lw=1.4, ls=(0, (3, 2))))
    ax.text(x, y, text, ha='center', va='center', fontsize=fs, **FONT)


def process(ax, x, y, w, h, text, fs=9):
    """이중 테두리 처리 상자."""
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle='round,pad=0,rounding_size=0.008', fc=FILL, ec=NAVY, lw=1.8))
    g = 0.008
    ax.add_patch(FancyBboxPatch((x - w / 2 + g, y - h / 2 + g), w - 2 * g, h - 2 * g,
                                boxstyle='round,pad=0,rounding_size=0.005', fc='none', ec=NAVY, lw=0.8))
    ax.text(x, y, text, ha='center', va='center', fontsize=fs, linespacing=1.25, **FONT)


def arrow(ax, x0, y0, x1, y1):
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.0, mutation_scale=9,
                                shrinkA=0, shrinkB=0))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'fig_multi_lidar_pipeline.png'
    fig = plt.figure(figsize=(4.4, 5.2))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0.14, 1.32)
    ax.axis('off')

    # 입력: LiDAR 3대
    in_y, in_h, in_w = 1.255, 0.085, 0.27
    inputs = [(0.165, 'Fixed\nLiDAR 1'), (0.5, 'Fixed\nLiDAR 2'), (0.835, 'Onboard\nMID-360')]
    for x, t in inputs:
        terminal(ax, x, in_y, in_w, in_h, t)

    # 고장 허용 융합
    fu_y, fu_h = 1.06, 0.11
    process(ax, 0.5, fu_y, 0.86, fu_h, 'Fault-tolerant fusion\n(stale source > 0.3 s excluded)')
    for x, _ in inputs:
        arrow(ax, x, in_y - in_h / 2, x, fu_y + fu_h / 2)

    # 이후 단계 (기존 Fig. 4와 동일)
    w, h = 0.56, 0.1
    steps = [
        (0.87, 'Fused point cloud', 'terminal'),
        (0.70, 'Height filter + self-\npoint removal', 'process'),
        (0.54, 'DBSCAN clustering', 'process'),
        (0.38, 'Target-candidate filter\n(size/height criteria)', 'process'),
        (0.20, 'Tracking', 'terminal'),
    ]
    prev_bottom = fu_y - fu_h / 2
    for y, text, kind in steps:
        hh = 0.085 if kind == 'terminal' else h
        arrow(ax, 0.5, prev_bottom, 0.5, y + hh / 2)
        (terminal if kind == 'terminal' else process)(ax, 0.5, y, w, hh, text)
        prev_bottom = y - hh / 2

    fig.savefig(out, dpi=300, facecolor='white', bbox_inches='tight', pad_inches=0.08)
    print(f'저장: {out}')


if __name__ == '__main__':
    main()
