#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 场地建模 + 路径规划 + 路径可视化（左半场）

功能：
1) 规则尺寸地图建模（12000x12000, 左半场）
2) 栅格 A* 规划（避障）
3) 路径平滑（可选）
4) 可视化：
   - 优先 MuJoCo viewer（若安装 mujoco）
   - 否则 matplotlib 2D 兜底

运行：
python tools/mujoco_map_planner.py

依赖：
pip install numpy matplotlib
可选：pip install mujoco
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
# 1) 你主要改这里
# =============================================================================

FIELD_W = 12000.0
FIELD_H = 12000.0
HALF_W = 6000.0

# 左半场
OUR_X0 = 0.0
OUR_X1 = HALF_W

ZONE1_H = 2000.0
ZONE2_H = 7300.0
ZONE3_H = 2700.0

# 方块区（项目内常用）
STAIRS_X = [290.0, 1490.0, 2690.0]
STAIRS_Y = [3290.0, 4490.0, 5690.0, 6890.0]
BLOCK_HALF = 290.0

# 起终点（mm）
START = (1200.0, 900.0)
GOAL = (2600.0, 10800.0)

# 栅格分辨率（mm/格）
GRID_RES = 100.0

# 机器人膨胀半径（mm），用于障碍膨胀
ROBOT_RADIUS = 420.0

# A* 4或8邻接
USE_DIAGONAL = True

# 平滑迭代次数（0表示不平滑）
SMOOTH_ITERS = 120

# MuJoCo 可视化参数
WORLD_SCALE = 0.001  # mm -> m
PATH_HEIGHT = 0.03   # m


# =============================================================================
# 2) 基础几何
# =============================================================================

Point = Tuple[float, float]
Rect = Tuple[float, float, float, float]  # x0,y0,w,h


@dataclass
class FieldModel:
    named_regions: Dict[str, Rect]
    forbidden_rects: List[Rect]
    reachable_rects: List[Rect]


def rect_contains(r: Rect, p: Point) -> bool:
    x0, y0, w, h = r
    return x0 <= p[0] <= x0 + w and y0 <= p[1] <= y0 + h


def build_field_model() -> FieldModel:
    y1 = ZONE1_H
    y2 = ZONE1_H + ZONE2_H

    regions: Dict[str, Rect] = {
        "一区(武馆MC)": (OUR_X0, 0.0, OUR_X1 - OUR_X0, ZONE1_H),
        "二区(梅林MF)": (OUR_X0, y1, OUR_X1 - OUR_X0, ZONE2_H),
        "三区(对抗区CF)": (OUR_X0, y2, OUR_X1 - OUR_X0, ZONE3_H),
    }

    forest_x0 = min(STAIRS_X) - BLOCK_HALF
    forest_x1 = max(STAIRS_X) + BLOCK_HALF
    forest_y0 = min(STAIRS_Y) - BLOCK_HALF
    forest_y1 = max(STAIRS_Y) + BLOCK_HALF
    regions["梅林-树林"] = (forest_x0, forest_y0, forest_x1 - forest_x0, forest_y1 - forest_y0)
    regions["梅林-R2入口区"] = (forest_x0, y1, forest_x1 - forest_x0, forest_y0 - y1)
    regions["梅林-R2出口区"] = (forest_x0, forest_y1, forest_x1 - forest_x0, y2 - forest_y1)

    regions["R2启动区"] = (250.0, 220.0, 1000.0, 800.0)
    regions["R1启动区"] = (250.0, 1120.0, 1000.0, 800.0)
    regions["长杆架"] = (1450.0, 380.0, 500.0, 300.0)
    regions["端头架"] = (5500.0 - 250.0, 820.0, 500.0, 300.0)

    regions["坡道-R2"] = (100.0, y2 + 120.0, 1300.0, 1500.0)
    regions["坡道-R1"] = (OUR_X1 - 1400.0, y2 + 120.0, 1300.0, 1500.0)
    regions["九宫格"] = ((OUR_X1 - 1620.0) / 2.0, y2 + 720.0, 1620.0, 1620.0)
    regions["已用兵器区"] = (180.0, y2 + 1850.0, 1500.0, 450.0)

    # R2可达（粗模型）
    reachable = [
        regions["一区(武馆MC)"],
        regions["梅林-R2入口区"],
        regions["梅林-树林"],
        regions["梅林-R2出口区"],
        regions["三区(对抗区CF)"],
    ]

    # 禁行（中隔板 + 可选额外障碍）
    forbidden = [
        (HALF_W - 25.0, 0.0, 50.0, FIELD_H),
    ]

    return FieldModel(named_regions=regions, forbidden_rects=forbidden, reachable_rects=reachable)


# =============================================================================
# 3) A* 栅格规划
# =============================================================================

def world_to_grid(p: Point, res: float) -> Tuple[int, int]:
    return int(round(p[0] / res)), int(round(p[1] / res))


def grid_to_world(g: Tuple[int, int], res: float) -> Point:
    return g[0] * res, g[1] * res


def point_in_any_rect(p: Point, rects: Sequence[Rect]) -> bool:
    return any(rect_contains(r, p) for r in rects)


def inflate_rect(r: Rect, d: float) -> Rect:
    x0, y0, w, h = r
    return x0 - d, y0 - d, w + 2.0 * d, h + 2.0 * d


def build_occupancy(model: FieldModel, res: float, robot_r: float) -> Tuple[np.ndarray, Tuple[int, int]]:
    w = int(FIELD_W / res) + 1
    h = int(FIELD_H / res) + 1
    occ = np.ones((h, w), dtype=np.uint8)  # 1=occupied

    inflated_forbidden = [inflate_rect(r, robot_r) for r in model.forbidden_rects]

    for gy in range(h):
        for gx in range(w):
            p = grid_to_world((gx, gy), res)
            in_left_half = (OUR_X0 <= p[0] <= OUR_X1)
            in_reach = point_in_any_rect(p, model.reachable_rects)
            in_forbid = point_in_any_rect(p, inflated_forbidden)
            occ[gy, gx] = 0 if (in_left_half and in_reach and not in_forbid) else 1

    return occ, (w, h)


def astar(occ: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], diagonal: bool) -> List[Tuple[int, int]]:
    h, w = occ.shape

    def in_bounds(x: int, y: int) -> bool:
        return 0 <= x < w and 0 <= y < h

    if diagonal:
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    else:
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)

    pq: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(pq, (0.0, start))
    g_cost = {start: 0.0}
    came: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            break

        for dx, dy in nbrs:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not in_bounds(nx, ny):
                continue
            if occ[ny, nx] == 1:
                continue
            step = math.hypot(dx, dy)
            ng = g_cost[cur] + step
            nxt = (nx, ny)
            if ng < g_cost.get(nxt, 1e30):
                g_cost[nxt] = ng
                came[nxt] = cur
                f = ng + heuristic(nxt, goal)
                heapq.heappush(pq, (f, nxt))

    if goal not in came and goal != start:
        return []

    path = [goal]
    while path[-1] != start:
        path.append(came[path[-1]])
    path.reverse()
    return path


def smooth_path(points: np.ndarray, iters: int) -> np.ndarray:
    if len(points) < 3 or iters <= 0:
        return points
    p = points.copy()
    for _ in range(iters):
        p[1:-1] = 0.5 * p[1:-1] + 0.25 * (p[:-2] + p[2:])
    return p


# =============================================================================
# 4) MuJoCo 场景生成与显示
# =============================================================================

def rect_center_size_m(r: Rect) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    x0, y0, w, h = r
    cx = (x0 + w * 0.5) * WORLD_SCALE
    cy = (y0 + h * 0.5) * WORLD_SCALE
    sx = max(w * 0.5 * WORLD_SCALE, 1e-4)
    sy = max(h * 0.5 * WORLD_SCALE, 1e-4)
    return (cx, cy, 0.0), (sx, sy, 0.01)


def build_mujoco_xml(model: FieldModel, path_xy: np.ndarray) -> str:
    geoms = []

    # 地面（左半场）
    ground = (OUR_X0, 0.0, OUR_X1 - OUR_X0, FIELD_H)
    c, s = rect_center_size_m(ground)
    geoms.append(f'<geom type="box" pos="{c[0]} {c[1]} -0.01" size="{s[0]} {s[1]} 0.01" rgba="0.85 0.85 0.85 1"/>')

    color_map = {
        "一区(武馆MC)": "0.80 0.93 0.80 0.35",
        "二区(梅林MF)": "0.98 0.96 0.72 0.35",
        "三区(对抗区CF)": "0.74 0.85 0.98 0.35",
        "梅林-树林": "1.00 0.80 0.74 0.45",
        "梅林-R2入口区": "1.00 0.89 0.51 0.45",
        "梅林-R2出口区": "1.00 0.89 0.51 0.45",
        "R2启动区": "0.78 0.90 0.79 0.60",
        "九宫格": "1.00 0.80 0.82 0.55",
        "已用兵器区": "0.85 0.80 0.78 0.55",
        "坡道-R2": "0.70 0.87 0.86 0.55",
        "坡道-R1": "0.70 0.87 0.86 0.55",
        "长杆架": "0.88 0.75 0.91 0.60",
        "端头架": "0.97 0.73 0.84 0.60",
        "R1启动区": "0.77 0.79 0.91 0.60",
    }

    for name, r in model.named_regions.items():
        c, s = rect_center_size_m(r)
        rgba = color_map.get(name, "0.9 0.9 0.9 0.4")
        geoms.append(f'<geom type="box" pos="{c[0]} {c[1]} 0.0" size="{s[0]} {s[1]} 0.002" rgba="{rgba}"/>')

    # 禁行（中隔板）
    for r in model.forbidden_rects:
        c, s = rect_center_size_m(r)
        geoms.append(f'<geom type="box" pos="{c[0]} {c[1]} 0.03" size="{s[0]} {s[1]} 0.03" rgba="0.85 0.20 0.20 0.60"/>')

    # 路径画成 capsule chain
    path_geoms = []
    if len(path_xy) >= 2:
        for i in range(len(path_xy) - 1):
            p0 = path_xy[i] * WORLD_SCALE
            p1 = path_xy[i + 1] * WORLD_SCALE
            path_geoms.append(
                f'<geom type="capsule" fromto="{p0[0]} {p0[1]} {PATH_HEIGHT} {p1[0]} {p1[1]} {PATH_HEIGHT}" size="0.015" rgba="0.12 0.35 0.92 1"/>'
            )

    sx, sy = START[0] * WORLD_SCALE, START[1] * WORLD_SCALE
    gx, gy = GOAL[0] * WORLD_SCALE, GOAL[1] * WORLD_SCALE

    xml = f"""
<mujoco model="r2_field_path">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <visual>
    <map znear="0.001"/>
    <quality shadowsize="2048"/>
  </visual>
  <worldbody>
    {''.join(geoms)}
    {''.join(path_geoms)}
    <geom type="sphere" pos="{sx} {sy} 0.04" size="0.04" rgba="0.0 0.8 0.2 1"/>
    <geom type="sphere" pos="{gx} {gy} 0.04" size="0.05" rgba="0.85 0.0 0.0 1"/>
    <light pos="6 6 8" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <camera name="top" pos="3 6 12" xyaxes="1 0 0 0 1 0"/>
  </worldbody>
</mujoco>
"""
    return xml


def show_mujoco_if_available(model: FieldModel, path_xy: np.ndarray) -> bool:
    try:
        import mujoco
        import mujoco.viewer
    except Exception:
        return False

    xml = build_mujoco_xml(model, path_xy)
    mj_model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(mj_model)

    with mujoco.viewer.launch_passive(mj_model, data) as viewer:
        viewer.cam.type = 1
        viewer.cam.fixedcamid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "top")
        # 纯静态场景，循环保持窗口
        while viewer.is_running():
            mujoco.mj_step(mj_model, data)
            viewer.sync()
    return True


# =============================================================================
# 5) 2D 兜底可视化
# =============================================================================

def show_matplotlib(model: FieldModel, occ: np.ndarray, path_xy: np.ndarray) -> None:
    plt.figure(figsize=(11, 9))

    # occupancy
    ys, xs = np.where(occ == 1)
    if len(xs) > 0:
        wx = xs * GRID_RES
        wy = ys * GRID_RES
        plt.scatter(wx, wy, s=2, c="#ef9a9a", alpha=0.35, label="障碍/禁行")

    # regions
    for name, r in model.named_regions.items():
        x0, y0, w, h = r
        plt.gca().add_patch(plt.Rectangle((x0, y0), w, h, fill=False, linewidth=1.0, edgecolor="#555"))

    if len(path_xy) > 0:
        plt.plot(path_xy[:, 0], path_xy[:, 1], "-", lw=2.4, c="#1565c0", label="规划路径")

    plt.scatter([START[0]], [START[1]], c="#00c853", s=60, label="起点")
    plt.scatter([GOAL[0]], [GOAL[1]], c="#d50000", s=70, marker="*", label="终点")

    plt.title("左半场路径规划可视化（2D兜底）")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.xlim(OUR_X0 - 200, OUR_X1 + 200)
    plt.ylim(FIELD_H + 200, -200)
    plt.gca().set_aspect("equal")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


# =============================================================================
# 6) 主流程
# =============================================================================

def main() -> None:
    model = build_field_model()
    occ, _ = build_occupancy(model, GRID_RES, ROBOT_RADIUS)

    s = world_to_grid(START, GRID_RES)
    g = world_to_grid(GOAL, GRID_RES)

    if occ[s[1], s[0]] == 1:
        raise RuntimeError("起点在障碍/禁行区域，请修改 START 或地图参数")
    if occ[g[1], g[0]] == 1:
        raise RuntimeError("终点在障碍/禁行区域，请修改 GOAL 或地图参数")

    path_g = astar(occ, s, g, diagonal=USE_DIAGONAL)
    if not path_g:
        raise RuntimeError("A* 未找到路径，请调整障碍/分辨率/起终点")

    path_xy = np.array([grid_to_world(p, GRID_RES) for p in path_g], dtype=float)
    path_xy = smooth_path(path_xy, SMOOTH_ITERS)

    length = float(np.sum(np.hypot(np.diff(path_xy[:, 0]), np.diff(path_xy[:, 1]))))
    print(f"[info] path points={len(path_xy)}, length={length:.1f} mm")

    shown = show_mujoco_if_available(model, path_xy)
    if not shown:
        print("[info] 未检测到 mujoco，使用 matplotlib 2D 显示")
        show_matplotlib(model, occ, path_xy)


if __name__ == "__main__":
    main()
