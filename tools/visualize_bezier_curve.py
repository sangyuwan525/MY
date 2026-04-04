#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化“真实 bezier trace_type”轨迹（非离散直线段）。

主要展示：
1. 连续三次贝塞尔曲线
2. 控制多边形和控制点
3. 曲线上切向方向箭头
4. 曲率热度（颜色越亮曲率越大）

示例：
python tools/visualize_bezier_curve.py ^
  --p0 0 0 --p1 800 0 --p2 1200 1200 --p3 2000 1000
"""

import argparse
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np


Point = Tuple[float, float]


def bezier_point(p0: Point, p1: Point, p2: Point, p3: Point, t: np.ndarray) -> np.ndarray:
    u = 1.0 - t
    return (
        (u ** 3)[:, None] * np.array(p0)[None, :]
        + (3.0 * (u ** 2) * t)[:, None] * np.array(p1)[None, :]
        + (3.0 * u * (t ** 2))[:, None] * np.array(p2)[None, :]
        + (t ** 3)[:, None] * np.array(p3)[None, :]
    )


def bezier_d1(p0: Point, p1: Point, p2: Point, p3: Point, t: np.ndarray) -> np.ndarray:
    u = 1.0 - t
    return (
        (3.0 * (u ** 2))[:, None] * (np.array(p1) - np.array(p0))[None, :]
        + (6.0 * u * t)[:, None] * (np.array(p2) - np.array(p1))[None, :]
        + (3.0 * (t ** 2))[:, None] * (np.array(p3) - np.array(p2))[None, :]
    )


def bezier_d2(p0: Point, p1: Point, p2: Point, p3: Point, t: np.ndarray) -> np.ndarray:
    u = 1.0 - t
    return (
        (6.0 * u)[:, None] * (np.array(p2) - 2.0 * np.array(p1) + np.array(p0))[None, :]
        + (6.0 * t)[:, None] * (np.array(p3) - 2.0 * np.array(p2) + np.array(p1))[None, :]
    )


def curvature(p0: Point, p1: Point, p2: Point, p3: Point, t: np.ndarray) -> np.ndarray:
    d1 = bezier_d1(p0, p1, p2, p3, t)
    d2 = bezier_d2(p0, p1, p2, p3, t)
    cross = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    denom = np.linalg.norm(d1, axis=1) ** 3
    kappa = np.zeros_like(cross)
    mask = denom > 1e-9
    kappa[mask] = cross[mask] / denom[mask]
    return kappa


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="可视化三次贝塞尔（trace_type=bezier）")
    parser.add_argument("--p0", type=float, nargs=2, required=True, metavar=("X", "Y"), help="起点 P0")
    parser.add_argument("--p1", type=float, nargs=2, required=True, metavar=("X", "Y"), help="控制点 P1")
    parser.add_argument("--p2", type=float, nargs=2, required=True, metavar=("X", "Y"), help="控制点 P2")
    parser.add_argument("--p3", type=float, nargs=2, required=True, metavar=("X", "Y"), help="终点 P3")
    parser.add_argument("--samples", type=int, default=400, help="曲线采样点数")
    parser.add_argument("--tangent-count", type=int, default=12, help="显示切向箭头数量")
    parser.add_argument("--save", type=str, default="", help="保存图片路径（可选）")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    samples = max(80, args.samples)
    tangent_count = max(3, args.tangent_count)

    p0: Point = (args.p0[0], args.p0[1])
    p1: Point = (args.p1[0], args.p1[1])
    p2: Point = (args.p2[0], args.p2[1])
    p3: Point = (args.p3[0], args.p3[1])
    ctrl = np.array([p0, p1, p2, p3], dtype=float)

    t = np.linspace(0.0, 1.0, samples)
    pts = bezier_point(p0, p1, p2, p3, t)
    d1 = bezier_d1(p0, p1, p2, p3, t)
    kappa = curvature(p0, p1, p2, p3, t)

    fig, ax = plt.subplots(figsize=(8, 8))

    kappa_norm = kappa / (np.max(kappa) + 1e-9)
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=kappa_norm, cmap="turbo", s=10, label="Bezier curve (curvature heat)")
    fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04, label="normalized curvature")

    ax.plot(ctrl[:, 0], ctrl[:, 1], "k--", linewidth=1.0, alpha=0.6, label="Control polygon")
    ax.scatter(ctrl[:, 0], ctrl[:, 1], s=50, c=["tab:green", "tab:orange", "tab:orange", "tab:red"], zorder=5)
    for i, p in enumerate(ctrl):
        ax.text(p[0], p[1], f"  P{i}", fontsize=10)

    t_idx = np.linspace(0, samples - 1, tangent_count, dtype=int)
    tan = d1[t_idx]
    tan_norm = np.linalg.norm(tan, axis=1, keepdims=True)
    tan_norm[tan_norm < 1e-9] = 1.0
    tan_u = tan / tan_norm
    arrow_len = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) * 0.04
    ax.quiver(
        pts[t_idx, 0],
        pts[t_idx, 1],
        tan_u[:, 0] * arrow_len,
        tan_u[:, 1] * arrow_len,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.003,
        color="black",
        alpha=0.75,
        label="Tangent direction",
    )

    ax.set_title("Bezier Trace-Type Visualization")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"[info] saved figure -> {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
