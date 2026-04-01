#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按工程中 init_tangent_line_circle_path 的几何逻辑可视化“直线 + 圆弧”路径。

输入参数与 C 函数一致：
    line_start, arc_end, center, arc_ccw

示例：
python tools/visualize_tangent_line_circle.py --line-start 0 0 --arc-end 1000 800 --center 1200 1200 --arc-ccw 1
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt


@dataclass
class Point:
    x: float
    y: float


def vec_norm(x: float, y: float) -> float:
    return math.hypot(x, y)


def normalize_angle_positive(angle: float) -> float:
    """将角度归一到 [0, 2*pi)。"""
    while angle < 0.0:
        angle += 2.0 * math.pi
    while angle >= 2.0 * math.pi:
        angle -= 2.0 * math.pi
    return angle


def get_arc_angle_by_direction(start: Point, end: Point, center: Point, arc_ccw: int) -> float:
    """
    与 C 代码一致：
    - arc_ccw=1: 返回逆时针跨越角
    - arc_ccw=0: 返回顺时针跨越角（以正值表示）
    """
    start_angle = math.atan2(start.y - center.y, start.x - center.x)
    end_angle = math.atan2(end.y - center.y, end.x - center.x)
    ccw_delta = normalize_angle_positive(end_angle - start_angle)
    if arc_ccw:
        return ccw_delta
    return (2.0 * math.pi) if ccw_delta < 1e-6 else (2.0 * math.pi - ccw_delta)


def select_tangent_point(
    line_start: Point, arc_end: Point, center: Point, arc_ccw: int
) -> Tuple[Point, float]:
    """
    复现 path.c 里的 select_tangent_point：
    从 line_start 向圆作两条候选切线，按“线段方向与圆弧切向一致（dot > 0.99）”选唯一切点。
    返回：tangent_point, central_angle
    """
    radius = vec_norm(arc_end.x - center.x, arc_end.y - center.y)
    vx = line_start.x - center.x
    vy = line_start.y - center.y
    d2 = vx * vx + vy * vy
    r2 = radius * radius

    if radius < 1e-6 or d2 <= r2 + 1e-6:
        raise ValueError("line_start 在圆内/圆上，无法构造切线。")

    base = r2 / d2
    factor = radius * math.sqrt(d2 - r2) / d2
    perp = (-vy, vx)

    candidates = [
        Point(center.x + base * vx + factor * perp[0], center.y + base * vy + factor * perp[1]),
        Point(center.x + base * vx - factor * perp[0], center.y + base * vy - factor * perp[1]),
    ]

    for c in candidates:
        line_dir_x = c.x - line_start.x
        line_dir_y = c.y - line_start.y
        radius_dir_x = c.x - center.x
        radius_dir_y = c.y - center.y

        if arc_ccw:
            arc_tan_x = -radius_dir_y
            arc_tan_y = radius_dir_x
        else:
            arc_tan_x = radius_dir_y
            arc_tan_y = -radius_dir_x

        line_n = vec_norm(line_dir_x, line_dir_y)
        tan_n = vec_norm(arc_tan_x, arc_tan_y)
        if line_n < 1e-6 or tan_n < 1e-6:
            continue

        line_dir_x /= line_n
        line_dir_y /= line_n
        arc_tan_x /= tan_n
        arc_tan_y /= tan_n
        dot = line_dir_x * arc_tan_x + line_dir_y * arc_tan_y

        if dot > 0.99:
            central_angle = get_arc_angle_by_direction(c, arc_end, center, arc_ccw)
            return c, central_angle

    raise ValueError("未找到满足切向连续的切点。")


def sample_arc_points(start: Point, center: Point, central_angle: float, arc_ccw: int, n: int = 200) -> List[Point]:
    """按切点为起点、给定方向与跨越角采样圆弧点。"""
    radius = vec_norm(start.x - center.x, start.y - center.y)
    start_ang = math.atan2(start.y - center.y, start.x - center.x)

    if arc_ccw:
        angles = [start_ang + central_angle * i / (n - 1) for i in range(n)]
    else:
        angles = [start_ang - central_angle * i / (n - 1) for i in range(n)]

    return [Point(center.x + radius * math.cos(a), center.y + radius * math.sin(a)) for a in angles]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="可视化 init_tangent_line_circle_path 生成的直线+圆弧路径")
    p.add_argument("--line-start", nargs=2, type=float, required=True, metavar=("X", "Y"), help="line_start")
    p.add_argument("--arc-end", nargs=2, type=float, required=True, metavar=("X", "Y"), help="arc_end")
    p.add_argument("--center", nargs=2, type=float, required=True, metavar=("X", "Y"), help="center")
    p.add_argument("--arc-ccw", type=int, choices=[0, 1], required=True, help="1=逆时针, 0=顺时针")
    p.add_argument("--save", type=str, default="", help="保存图片路径（可选）")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    line_start = Point(*args.line_start)
    arc_end = Point(*args.arc_end)
    center = Point(*args.center)
    arc_ccw = int(args.arc_ccw)

    tangent, central_angle = select_tangent_point(line_start, arc_end, center, arc_ccw)
    radius = vec_norm(arc_end.x - center.x, arc_end.y - center.y)
    arc_pts = sample_arc_points(tangent, center, central_angle, arc_ccw, n=220)

    fig, ax = plt.subplots(figsize=(8, 8))

    # 整圆参考
    circle = plt.Circle((center.x, center.y), radius, fill=False, linestyle="--", linewidth=1.0, alpha=0.5)
    ax.add_patch(circle)

    # 直线段（line_start -> tangent）
    ax.plot([line_start.x, tangent.x], [line_start.y, tangent.y], linewidth=2.5, label="Line Segment")

    # 圆弧段（tangent -> arc_end）
    ax.plot([p.x for p in arc_pts], [p.y for p in arc_pts], linewidth=2.5, label="Arc Segment")

    # 关键点
    ax.scatter([line_start.x], [line_start.y], s=55, label="line_start")
    ax.scatter([tangent.x], [tangent.y], s=55, label="tangent_point")
    ax.scatter([arc_end.x], [arc_end.y], s=55, label="arc_end")
    ax.scatter([center.x], [center.y], s=55, marker="x", label="center")

    # 切向方向箭头（用于看是否与 arc_ccw 一致）
    rdx = tangent.x - center.x
    rdy = tangent.y - center.y
    if arc_ccw:
        tx, ty = -rdy, rdx
    else:
        tx, ty = rdy, -rdx
    tn = vec_norm(tx, ty)
    if tn > 1e-6:
        tx, ty = tx / tn, ty / tn
        scale = max(radius * 0.15, 80.0)
        ax.arrow(tangent.x, tangent.y, tx * scale, ty * scale, width=0.0, head_width=scale * 0.18, length_includes_head=True)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(
        f"Tangent Line + Arc | arc_ccw={arc_ccw}\n"
        f"tangent=({tangent.x:.2f}, {tangent.y:.2f}), central_angle={central_angle:.4f} rad"
    )
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    if args.save:
        fig.savefig(args.save, dpi=160, bbox_inches="tight")
        print(f"[Saved] {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

