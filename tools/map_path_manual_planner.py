#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
R2 左半场地图建模 + 手动路径规划（规则尺寸）

规则关键尺寸：
- 全场：12000 x 12000 mm
- 三区纵向：一区2000、二区7300、三区2700
- 中轴隔板厚：50 mm
- 启动区：1000 x 800 mm
- 九宫格：1620 x 1620 mm

特点：
- 固定左半场（x in [0, 6000]）
- 支持 line / arc / bezier 手动路径
- 中文标注采用“图内编号 + 右侧中文说明”，尽量不遮挡地图
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Literal, Sequence, Tuple
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


Point = Tuple[float, float]
SegmentType = Literal["line", "arc", "bezier"]


FIELD_W = 12000.0
FIELD_H = 12000.0

ZONE1_H = 2000.0
ZONE2_H = 7300.0
ZONE3_H = 2700.0

HALF_W = 6000.0
CENTER_WALL_THICK = 50.0
CENTER_WALL_X0 = HALF_W - CENTER_WALL_THICK / 2.0
CENTER_WALL_X1 = HALF_W + CENTER_WALL_THICK / 2.0

OUR_X0 = 0.0
OUR_X1 = HALF_W

START_POINT: Point = (1200.0, 900.0)
END_POINT: Point = (2600.0, 10800.0)
END_YAW_RAD = -math.pi / 2.0

STAIRS_X = [290.0, 1490.0, 2690.0]
STAIRS_Y = [3290.0, 4490.0, 5690.0, 6890.0]
BLOCK_HALF = 290.0

START_ZONE_W = 1000.0
START_ZONE_H = 800.0
JIUGONG_W = 1620.0
JIUGONG_H = 1620.0

PATH_SEGMENTS: List[Dict] = [
    {"type": "line", "start": START_POINT, "end": (2300.0, 2100.0)},
    {"type": "bezier", "p0": (2300.0, 2100.0), "p1": (2200.0, 4200.0), "p2": (2100.0, 6800.0), "p3": (2300.0, 8200.0)},
    {"type": "arc", "center": (2600.0, 9300.0), "radius": 600.0, "start_angle": math.radians(180.0), "end_angle": math.radians(315.0), "ccw": True},
    {"type": "line", "start": (3024.0, 8876.0), "end": END_POINT},
]

LINE_SAMPLES = 60
ARC_SAMPLES = 120
BEZIER_SAMPLES = 180


@dataclass
class LineSeg:
    start: Point
    end: Point


@dataclass
class ArcSeg:
    center: Point
    radius: float
    start_angle: float
    end_angle: float
    ccw: bool


@dataclass
class BezierSeg:
    p0: Point
    p1: Point
    p2: Point
    p3: Point


def rect(x0: float, y0: float, w: float, h: float) -> List[Point]:
    return [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]


def normalize_2pi(angle: float) -> float:
    while angle < 0.0:
        angle += 2.0 * math.pi
    while angle >= 2.0 * math.pi:
        angle -= 2.0 * math.pi
    return angle


def line_points(seg: LineSeg, n: int) -> np.ndarray:
    t = np.linspace(0.0, 1.0, max(2, n))
    x = seg.start[0] + (seg.end[0] - seg.start[0]) * t
    y = seg.start[1] + (seg.end[1] - seg.start[1]) * t
    return np.column_stack([x, y])


def arc_points(seg: ArcSeg, n: int) -> np.ndarray:
    a0 = normalize_2pi(seg.start_angle)
    a1 = normalize_2pi(seg.end_angle)
    if seg.ccw:
        sweep = a1 - a0
        if sweep < 0.0:
            sweep += 2.0 * math.pi
        ang = np.linspace(a0, a0 + sweep, max(2, n))
    else:
        sweep = a0 - a1
        if sweep < 0.0:
            sweep += 2.0 * math.pi
        ang = np.linspace(a0, a0 - sweep, max(2, n))
    x = seg.center[0] + seg.radius * np.cos(ang)
    y = seg.center[1] + seg.radius * np.sin(ang)
    return np.column_stack([x, y])


def bezier_points(seg: BezierSeg, n: int) -> np.ndarray:
    t = np.linspace(0.0, 1.0, max(2, n))
    u = 1.0 - t
    p0 = np.array(seg.p0)
    p1 = np.array(seg.p1)
    p2 = np.array(seg.p2)
    p3 = np.array(seg.p3)
    return (
        (u**3)[:, None] * p0[None, :]
        + (3.0 * (u**2) * t)[:, None] * p1[None, :]
        + (3.0 * u * (t**2))[:, None] * p2[None, :]
        + (t**3)[:, None] * p3[None, :]
    )


def polyline_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    d = np.diff(points, axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        inter = ((y1 > y) != (y2 > y))
        if inter:
            xin = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if x < xin:
                inside = not inside
    return inside


def in_any_polygon(point: Point, polygons: Sequence[Sequence[Point]]) -> bool:
    return any(point_in_polygon(point, p) for p in polygons)


def poly_center(poly: Sequence[Point]) -> Point:
    a = np.array(poly, dtype=float)
    return float(np.mean(a[:, 0])), float(np.mean(a[:, 1]))


def build_field_regions() -> Tuple[Dict[str, List[Point]], List[List[Point]], List[List[Point]]]:
    y1 = ZONE1_H
    y2 = ZONE1_H + ZONE2_H
    x0, x1 = OUR_X0, OUR_X1

    regions: Dict[str, List[Point]] = {
        "一区(武馆MC)": rect(x0, 0.0, x1 - x0, ZONE1_H),
        "二区(梅林MF)": rect(x0, y1, x1 - x0, ZONE2_H),
        "三区(对抗区CF)": rect(x0, y2, x1 - x0, ZONE3_H),
    }

    forest_x0 = min(STAIRS_X) - BLOCK_HALF
    forest_x1 = max(STAIRS_X) + BLOCK_HALF
    forest_y0 = min(STAIRS_Y) - BLOCK_HALF
    forest_y1 = max(STAIRS_Y) + BLOCK_HALF
    regions["梅林-树林(12桩)"] = rect(forest_x0, forest_y0, forest_x1 - forest_x0, forest_y1 - forest_y0)
    regions["梅林-R2入口区"] = rect(forest_x0, y1, forest_x1 - forest_x0, forest_y0 - y1)
    regions["梅林-R2出口区"] = rect(forest_x0, forest_y1, forest_x1 - forest_x0, y2 - forest_y1)

    sz_x = x0 + 250.0
    regions["R2启动区"] = rect(sz_x, 220.0, START_ZONE_W, START_ZONE_H)
    regions["R1启动区"] = rect(sz_x, 1120.0, START_ZONE_W, START_ZONE_H)
    regions["长杆架"] = rect(sz_x + 1200.0, 380.0, 500.0, 300.0)
    headrack_x = HALF_W - 250.0
    regions["端头架"] = rect(headrack_x - 250.0, 820.0, 500.0, 300.0)

    regions["坡道-R2"] = rect(x0 + 100.0, y2 + 120.0, 1300.0, 1500.0)
    regions["坡道-R1"] = rect(x1 - 1400.0, y2 + 120.0, 1300.0, 1500.0)
    jx = x0 + (x1 - x0 - JIUGONG_W) / 2.0
    jy = y2 + 720.0
    regions["九宫格"] = rect(jx, jy, JIUGONG_W, JIUGONG_H)
    regions["已用兵器区"] = rect(x0 + 180.0, y2 + 1850.0, 1500.0, 450.0)

    reachable = [
        regions["一区(武馆MC)"],
        regions["梅林-R2入口区"],
        regions["梅林-R2出口区"],
        regions["梅林-树林(12桩)"],
        regions["三区(对抗区CF)"],
    ]

    forbidden: List[List[Point]] = [
        rect(CENTER_WALL_X0, 0.0, CENTER_WALL_X1 - CENTER_WALL_X0, FIELD_H),
    ]

    return regions, reachable, forbidden


def build_segments(raw: Sequence[Dict]) -> List[Tuple[SegmentType, object]]:
    out: List[Tuple[SegmentType, object]] = []
    for i, item in enumerate(raw):
        tp = item.get("type")
        if tp == "line":
            out.append(("line", LineSeg(start=tuple(item["start"]), end=tuple(item["end"]))))
        elif tp == "arc":
            out.append(("arc", ArcSeg(center=tuple(item["center"]), radius=float(item["radius"]), start_angle=float(item["start_angle"]), end_angle=float(item["end_angle"]), ccw=bool(item["ccw"]))))
        elif tp == "bezier":
            out.append(("bezier", BezierSeg(p0=tuple(item["p0"]), p1=tuple(item["p1"]), p2=tuple(item["p2"]), p3=tuple(item["p3"]))))
        else:
            raise ValueError(f"PATH_SEGMENTS[{i}] type 非法: {tp}")
    return out


def sample_segment(seg_type: SegmentType, seg_obj: object) -> np.ndarray:
    if seg_type == "line":
        return line_points(seg_obj, LINE_SAMPLES)  # type: ignore[arg-type]
    if seg_type == "arc":
        return arc_points(seg_obj, ARC_SAMPLES)  # type: ignore[arg-type]
    if seg_type == "bezier":
        return bezier_points(seg_obj, BEZIER_SAMPLES)  # type: ignore[arg-type]
    raise ValueError(seg_type)


def check_reachability(path_points: np.ndarray, reachable: Sequence[Sequence[Point]], forbidden: Sequence[Sequence[Point]]) -> Tuple[int, int]:
    ok, bad = 0, 0
    for p in path_points:
        pt = (float(p[0]), float(p[1]))
        if in_any_polygon(pt, reachable) and (not in_any_polygon(pt, forbidden)):
            ok += 1
        else:
            bad += 1
    return ok, bad


def setup_chinese_font() -> None:
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "WenQuanYi Zen Hei",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def draw_polygon(ax, poly: Sequence[Point], fc: str, ec: str, alpha: float, lw: float, label: str | None = None):
    arr = np.array(list(poly) + [poly[0]], dtype=float)
    ax.fill(arr[:, 0], arr[:, 1], facecolor=fc, edgecolor=ec, linewidth=lw, alpha=alpha, label=label)


def main() -> None:
    setup_chinese_font()
    regions, reachable, forbidden = build_field_regions()
    segments = build_segments(PATH_SEGMENTS)

    region_colors = {
        "一区(武馆MC)": ("#dcedc8", "#558b2f"),
        "二区(梅林MF)": ("#fff9c4", "#f9a825"),
        "三区(对抗区CF)": ("#bbdefb", "#1565c0"),
        "R2启动区": ("#c8e6c9", "#1b5e20"),
        "R1启动区": ("#c5cae9", "#303f9f"),
        "长杆架": ("#e1bee7", "#6a1b9a"),
        "端头架": ("#f8bbd0", "#ad1457"),
        "梅林-R2入口区": ("#ffe082", "#ff8f00"),
        "梅林-树林(12桩)": ("#ffccbc", "#d84315"),
        "梅林-R2出口区": ("#ffe082", "#ff8f00"),
        "坡道-R2": ("#b2dfdb", "#00695c"),
        "坡道-R1": ("#b2dfdb", "#00695c"),
        "九宫格": ("#ffcdd2", "#b71c1c"),
        "已用兵器区": ("#d7ccc8", "#4e342e"),
    }

    order_main = ["一区(武馆MC)", "二区(梅林MF)", "三区(对抗区CF)"]
    order_by_competition = [
        "R2启动区", "R1启动区", "长杆架", "端头架",
        "梅林-R2入口区", "梅林-树林(12桩)", "梅林-R2出口区",
        "坡道-R2", "坡道-R1", "九宫格", "已用兵器区",
    ]

    fig, ax = plt.subplots(figsize=(12.8, 9))
    fig.subplots_adjust(right=0.70)

    ax.set_title("R2 左半场地图（规则尺寸）与路径可视化")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.30)

    border = np.array([[0.0, 0.0], [FIELD_W, 0.0], [FIELD_W, FIELD_H], [0.0, FIELD_H], [0.0, 0.0]])
    ax.plot(border[:, 0], border[:, 1], color="#616161", lw=1.0, alpha=0.45)
    left_box = np.array([[OUR_X0, 0.0], [OUR_X1, 0.0], [OUR_X1, FIELD_H], [OUR_X0, FIELD_H], [OUR_X0, 0.0]])
    ax.plot(left_box[:, 0], left_box[:, 1], "k-", lw=1.3)

    for i, name in enumerate(order_main):
        fc, ec = region_colors[name]
        draw_polygon(ax, regions[name], fc, ec, 0.27 if i == 0 else 0.23, 1.0, "主区域" if i == 0 else None)

    for name in order_by_competition:
        fc, ec = region_colors[name]
        draw_polygon(ax, regions[name], fc, ec, 0.35, 1.0)

    # 图内编号：按比赛口径顺序固定
    id_map: Dict[str, int] = {}
    for i, name in enumerate(order_by_competition, start=1):
        id_map[name] = i
        cx, cy = poly_center(regions[name])
        ax.scatter([cx], [cy], s=55, c="#ffffff", edgecolors="#333333", zorder=6)
        ax.text(cx, cy, str(i), fontsize=8, ha="center", va="center", zorder=7)

    for i, poly in enumerate(reachable):
        arr = np.array(poly + [poly[0]], dtype=float)
        ax.plot(arr[:, 0], arr[:, 1], color="#2e7d32", lw=1.2, alpha=0.9, label="R2可达区域" if i == 0 else None)

    for i, poly in enumerate(forbidden):
        arr = np.array(poly + [poly[0]], dtype=float)
        ax.fill(arr[:, 0], arr[:, 1], color="#ef5350", alpha=0.18, edgecolor="#c62828", linewidth=1.0, label="禁行区域" if i == 0 else None)

    color_map = {"line": "#1565c0", "arc": "#6a1b9a", "bezier": "#ef6c00"}
    all_pts: List[np.ndarray] = []
    total_len = 0.0
    for i, (tp, obj) in enumerate(segments):
        pts = sample_segment(tp, obj)
        all_pts.append(pts)
        seg_len = polyline_length(pts)
        total_len += seg_len
        ax.plot(pts[:, 0], pts[:, 1], color=color_map[tp], lw=2.6, label=f"{tp}#{i}  长度={seg_len:.1f}mm")
        if tp == "bezier":
            bz = obj  # type: ignore[assignment]
            ctrl = np.array([bz.p0, bz.p1, bz.p2, bz.p3], dtype=float)
            ax.plot(ctrl[:, 0], ctrl[:, 1], "k--", lw=1.0, alpha=0.45)
            ax.scatter(ctrl[:, 0], ctrl[:, 1], s=24, c=["#1b5e20", "#f57c00", "#f57c00", "#b71c1c"], zorder=6)

    path_points = np.vstack(all_pts) if all_pts else np.zeros((0, 2), dtype=float)
    ok, bad = check_reachability(path_points, reachable, forbidden)

    ax.scatter([START_POINT[0]], [START_POINT[1]], c="#00c853", s=90, marker="o", zorder=7, label="起点")
    ax.scatter([END_POINT[0]], [END_POINT[1]], c="#d50000", s=120, marker="*", zorder=7, label=f"终点(角度={END_YAW_RAD:.3f})")

    info = (
        f"左半场：x ∈ [{OUR_X0:.0f}, {OUR_X1:.0f}]\n"
        f"分区高度：{ZONE1_H:.0f}/{ZONE2_H:.0f}/{ZONE3_H:.0f}\n"
        f"路径总长：{total_len:.1f} mm\n"
        f"可达采样点：{ok}\n"
        f"越界/禁区采样点：{bad}"
    )
    ax.text(0.015, 0.985, info, transform=ax.transAxes, va="top", ha="left", fontsize=9,
            bbox=dict(facecolor="white", alpha=0.82, edgecolor="#888"))

    # 右侧说明栏：固定比赛顺序
    side_text = "\n".join([
        "区域编号（比赛顺序）：",
        "武馆：",
        f"{id_map['R2启动区']}. R2启动区",
        f"{id_map['R1启动区']}. R1启动区",
        f"{id_map['长杆架']}. 长杆架",
        f"{id_map['端头架']}. 端头架",
        "梅林：",
        f"{id_map['梅林-R2入口区']}. R2入口区",
        f"{id_map['梅林-树林(12桩)']}. 树林(12桩)",
        f"{id_map['梅林-R2出口区']}. R2出口区",
        "对抗区：",
        f"{id_map['坡道-R2']}. 坡道-R2",
        f"{id_map['坡道-R1']}. 坡道-R1",
        f"{id_map['九宫格']}. 九宫格",
        f"{id_map['已用兵器区']}. 已用兵器区",
    ])
    fig.text(0.715, 0.92, side_text, ha="left", va="top", fontsize=10,
             bbox=dict(facecolor="#fafafa", edgecolor="#888", alpha=0.96))

    # 右侧颜色图例（每个区域固定颜色）
    legend_order = order_main + order_by_competition
    handles = [Patch(facecolor=region_colors[n][0], edgecolor=region_colors[n][1], label=n) for n in legend_order]
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.712, 0.41),
               frameon=True, framealpha=0.96, title="区域颜色图例", fontsize=8, title_fontsize=9)

    ax.set_xlim(OUR_X0 - 200.0, OUR_X1 + 200.0)
    ax.set_ylim(FIELD_H + 200.0, -200.0)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout(rect=[0.0, 0.0, 0.70, 1.0])
    plt.show()


if __name__ == "__main__":
    main()
