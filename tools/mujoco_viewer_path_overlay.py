#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo viewer 实时路径叠加可视化（支持自定义局部坐标系）。

当前默认局部坐标定义：
- 原点：右半场 R2 启动区（可配置）
- +X：从 R2 启动区指向 R1 启动区
- +Y：按右手定则由 +X 自动计算（Z 朝上）
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import List, Tuple

import numpy as np
import mujoco
import mujoco.viewer

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.mujoco_collision_path_planner as planner


# =============================================================================
# 你主要改这里
# =============================================================================

XML_PATH = Path("mjcf/robocon2026.xml")

PATH_Z_M = 0.03
PATH_POINT_RADIUS_M = 0.02
VIEWER_HZ = 60.0

SHOW_ASTAR = True
GRID_RES_MM = 50.0
ROBOT_RADIUS_MM = 420.0

# --------------------- 局部坐标系定义（关键） ---------------------
# 注意：下面两个点是“全场 mm 坐标”（不是局部坐标）。
# 你可按规则图册改成准确值。
R2_START_GLOBAL_MM = (400.0, 7400.0)  # 左侧红色一区 R2 启动区参考点（可继续微调）
R1_START_GLOBAL_MM = (400.0, 11500.0)  # 用于定义 +X 方向（从 R2 指向 R1）
AXIS_HANDEDNESS = "left"  # "left" 或 "right"

# A* 与手工路径输入，是否按“局部坐标”解释。
# True: 你填的点是局部坐标（原点在 R2 启动区）
# False: 你填的点是全场坐标
INPUT_USE_LOCAL_FRAME = True

# A* 起终点（单位 mm）
ASTAR_START_MM = (0.0, 0.0)
ASTAR_GOAL_MM = (0.0, 0.0)

# 坐标辅助层
SHOW_COORD_OVERLAY = True
COORD_Z_M = 0.045
COORD_POINT_RADIUS_M = 0.012
AXIS_LEN_MM = 1200.0
AXIS_ARROW_HEAD_MM = 220.0
HALF_FIELD_SPLIT_X_MM = 6000.0
BOUNDARY_SAMPLES = 180

# 手工路径候选（单位 mm）
# 当 INPUT_USE_LOCAL_FRAME=True 时，下面所有点按“局部坐标”解释。
PATH_CANDIDATES = [
    # 1) 直线
    {
        "name": "demo_line",
        "rgba": (1.0, 0.2, 0.2, 1.0),
        "segments": [
            {
                "type": "line",
                "start": (0.0, 0.0),
                "end": (2500.0, 1200.0),
                "samples": 80,
            }
        ],
    },

    # 2) 圆弧
    {
        "name": "demo_arc",
        "rgba": (0.2, 0.8, 0.2, 1.0),
        "segments": [
            {
                "type": "arc",
                "center": (1500.0, 1500.0),
                "radius": 1200.0,
                "start_angle": 3.1415926,
                "end_angle": 1.5707963,
                "ccw": False,
                "samples": 120,
            }
        ],
    },

    # 3) 直线 + 圆弧相切
    {
        "name": "demo_tangent_line_arc",
        "rgba": (1.0, 0.6, 0.0, 1.0),
        "segments": [
            {
                "type": "tangent_line_arc",
                "line_start": (0.0, 0.0),
                "arc_end": (2800.0, 3000.0),
                "center": (3800.0, 3000.0),
                "arc_ccw": True,
                "line_samples": 80,
                "arc_samples": 140,
            }
        ],
    },

    # 4) 贝塞尔曲线（三次）
    {
        "name": "demo_bezier",
        "rgba": (0.2, 0.5, 1.0, 1.0),
        "segments": [
            {
                "type": "bezier",
                "p0": (0.0, 0.0),
                "p1": (1200.0, 2500.0),
                "p2": (2800.0, -500.0),
                "p3": (4200.0, 2200.0),
                "samples": 140,
            }
        ],
    },
]


def resolve_xml_path() -> Path:
    candidates = [
        XML_PATH,
        Path("mjcf/robocon2026.xml"),
        Path("mjcf/scene_go2.xml"),
        Path("mjcf/scene_go1.xml"),
        Path("mjcf/scene_g1.xml"),
        Path("mjcf/scene_t1.xml"),
        Path("mjcf/scene_tron.xml"),
    ]
    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError("找不到可用场景 XML。请检查 mjcf 目录。")


def compute_mm_to_world_affine(tri_xyz: np.ndarray) -> Tuple[float, float, float, float]:
    xy = tri_xyz[:, :, :2].reshape(-1, 2)
    minx = float(xy[:, 0].min())
    maxy = float(xy[:, 1].max())
    maxx = float(xy[:, 0].max())
    miny = float(xy[:, 1].min())
    sx = planner.FIELD_W_MM / max(maxx - minx, 1e-9)
    sy = planner.FIELD_H_MM / max(maxy - miny, 1e-9)
    return minx, maxy, sx, sy


def mm_to_world_xy(points_mm: np.ndarray, minx: float, maxy: float, sx: float, sy: float) -> np.ndarray:
    out = np.zeros((len(points_mm), 2), dtype=np.float64)
    out[:, 0] = minx + points_mm[:, 0] / sx
    out[:, 1] = maxy - points_mm[:, 1] / sy
    return out


def build_local_frame() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回局部坐标系：origin, x_unit, y_unit（都在全场 mm 平面下）。"""
    origin = np.array(R2_START_GLOBAL_MM, dtype=np.float64)
    x_vec = np.array(R1_START_GLOBAL_MM, dtype=np.float64) - origin
    norm = float(np.linalg.norm(x_vec))
    if norm < 1e-6:
        raise ValueError("R2_START_GLOBAL_MM 与 R1_START_GLOBAL_MM 不能重合")
    x_unit = x_vec / norm
    if AXIS_HANDEDNESS.lower() == "right":
        # 右手定则：z 朝上时，y = z × x = [-x_y, x_x]
        y_unit = np.array([-x_unit[1], x_unit[0]], dtype=np.float64)
    else:
        # 左手定则：z 朝上时，y = - (z × x) = [x_y, -x_x]
        y_unit = np.array([x_unit[1], -x_unit[0]], dtype=np.float64)
    return origin, x_unit, y_unit


def local_to_global_mm(points_mm: np.ndarray, origin: np.ndarray, x_unit: np.ndarray, y_unit: np.ndarray) -> np.ndarray:
    """局部(mm) -> 全场(mm)"""
    out = np.zeros_like(points_mm, dtype=np.float64)
    out[:, 0] = origin[0] + points_mm[:, 0] * x_unit[0] + points_mm[:, 1] * y_unit[0]
    out[:, 1] = origin[1] + points_mm[:, 0] * x_unit[1] + points_mm[:, 1] * y_unit[1]
    return out


def point_to_global_mm(p: Tuple[float, float], origin: np.ndarray, x_unit: np.ndarray, y_unit: np.ndarray) -> Tuple[float, float]:
    arr = np.array([[p[0], p[1]]], dtype=np.float64)
    g = local_to_global_mm(arr, origin, x_unit, y_unit)[0]
    return (float(g[0]), float(g[1]))


def draw_sphere_point(viewer, x: float, y: float, z: float, radius: float, rgba: Tuple[float, float, float, float]) -> None:
    scn = viewer.user_scn
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        g,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, radius, radius], dtype=np.float64),
        np.array([x, y, z], dtype=np.float64),
        np.eye(3).reshape(-1),
        np.array(rgba, dtype=np.float32),
    )
    scn.ngeom += 1


def draw_world_line_points(viewer, p0: Tuple[float, float], p1: Tuple[float, float], z_m: float, radius_m: float, rgba, spacing_m: float = 0.05) -> None:
    p0a = np.array(p0, dtype=np.float64)
    p1a = np.array(p1, dtype=np.float64)
    length = float(np.linalg.norm(p1a - p0a))
    n = max(2, int(length / max(spacing_m, 1e-4)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        p = p0a * (1.0 - t) + p1a * t
        draw_sphere_point(viewer, float(p[0]), float(p[1]), z_m, radius_m, rgba)


def build_paths_global_mm(origin: np.ndarray, x_unit: np.ndarray, y_unit: np.ndarray, raster: planner.RasterResult | None):
    all_paths = []

    for candi in PATH_CANDIDATES:
        pts = planner.build_candidate_points(candi["segments"])
        if len(pts) <= 1:
            continue
        if INPUT_USE_LOCAL_FRAME:
            pts = local_to_global_mm(pts, origin, x_unit, y_unit)
        all_paths.append((candi["name"], candi.get("rgba", (0.95, 0.5, 0.0, 1.0)), pts))

    if SHOW_ASTAR and raster is not None:
        s = ASTAR_START_MM
        g = ASTAR_GOAL_MM
        if INPUT_USE_LOCAL_FRAME:
            s = point_to_global_mm(s, origin, x_unit, y_unit)
            g = point_to_global_mm(g, origin, x_unit, y_unit)
        astar = planner.astar_on_occ(raster, s, g, diagonal=True)
        if len(astar) > 1:
            all_paths.append(("astar", (0.10, 0.40, 0.85, 1.0), astar))

    return all_paths


def draw_coordinate_overlay(viewer, origin: np.ndarray, x_unit: np.ndarray, y_unit: np.ndarray, minx: float, maxy: float, sx: float, sy: float) -> None:
    if not SHOW_COORD_OVERLAY:
        return

    # 局部原点（即右半场 R2 启动区）
    o_mm = np.array([[origin[0], origin[1]]], dtype=np.float64)
    o_w = mm_to_world_xy(o_mm, minx, maxy, sx, sy)[0]
    draw_sphere_point(viewer, float(o_w[0]), float(o_w[1]), COORD_Z_M, COORD_POINT_RADIUS_M * 1.4, (1.0, 1.0, 0.0, 1.0))

    # 局部 +X / +Y 箭头（基于你定义的局部坐标）
    x_tip = origin + x_unit * AXIS_LEN_MM
    y_tip = origin + y_unit * AXIS_LEN_MM
    x_head_l = x_tip - x_unit * AXIS_ARROW_HEAD_MM + y_unit * (AXIS_ARROW_HEAD_MM * 0.35)
    x_head_r = x_tip - x_unit * AXIS_ARROW_HEAD_MM - y_unit * (AXIS_ARROW_HEAD_MM * 0.35)
    y_head_l = y_tip - y_unit * AXIS_ARROW_HEAD_MM + x_unit * (AXIS_ARROW_HEAD_MM * 0.35)
    y_head_r = y_tip - y_unit * AXIS_ARROW_HEAD_MM - x_unit * (AXIS_ARROW_HEAD_MM * 0.35)

    def to_world(p_mm: np.ndarray) -> Tuple[float, float]:
        return tuple(mm_to_world_xy(p_mm.reshape(1, 2), minx, maxy, sx, sy)[0])

    ow = to_world(origin)
    xw = to_world(x_tip)
    yw = to_world(y_tip)
    xhlw = to_world(x_head_l)
    xhrw = to_world(x_head_r)
    yhlw = to_world(y_head_l)
    yhrw = to_world(y_head_r)

    draw_world_line_points(viewer, ow, xw, COORD_Z_M, COORD_POINT_RADIUS_M, (1.0, 0.1, 0.1, 1.0))
    draw_world_line_points(viewer, xw, xhlw, COORD_Z_M, COORD_POINT_RADIUS_M, (1.0, 0.1, 0.1, 1.0))
    draw_world_line_points(viewer, xw, xhrw, COORD_Z_M, COORD_POINT_RADIUS_M, (1.0, 0.1, 0.1, 1.0))

    draw_world_line_points(viewer, ow, yw, COORD_Z_M, COORD_POINT_RADIUS_M, (0.1, 1.0, 0.1, 1.0))
    draw_world_line_points(viewer, yw, yhlw, COORD_Z_M, COORD_POINT_RADIUS_M, (0.1, 1.0, 0.1, 1.0))
    draw_world_line_points(viewer, yw, yhrw, COORD_Z_M, COORD_POINT_RADIUS_M, (0.1, 1.0, 0.1, 1.0))

    # 半场分界线（全场坐标 x=6000mm）
    ys = np.linspace(0.0, 12000.0, max(2, BOUNDARY_SAMPLES))
    xs = np.full_like(ys, HALF_FIELD_SPLIT_X_MM)
    split_mm = np.stack([xs, ys], axis=1)
    split_world = mm_to_world_xy(split_mm, minx, maxy, sx, sy)
    for i in range(len(split_world)):
        draw_sphere_point(
            viewer,
            float(split_world[i, 0]),
            float(split_world[i, 1]),
            COORD_Z_M,
            COORD_POINT_RADIUS_M * 0.8,
            (0.2, 0.9, 1.0, 0.95),
        )


def main() -> None:
    xml_path = resolve_xml_path()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    tri_xyz = planner.read_collision_triangles(xml_path)
    minx, maxy, sx, sy = compute_mm_to_world_affine(tri_xyz)

    origin, x_unit, y_unit = build_local_frame()
    print(f"[info] local_origin_global_mm=({origin[0]:.1f}, {origin[1]:.1f})")
    print(f"[info] local_x_unit=({x_unit[0]:.4f}, {x_unit[1]:.4f}), local_y_unit=({y_unit[0]:.4f}, {y_unit[1]:.4f})")

    raster = None
    if SHOW_ASTAR:
        tri_mm = planner.auto_metric_to_field(tri_xyz)
        raster = planner.rasterize_triangles(tri_mm, GRID_RES_MM, ROBOT_RADIUS_MM)

    paths_mm = build_paths_global_mm(origin, x_unit, y_unit, raster)
    print(f"[info] 显示路径数: {len(paths_mm)}")

    dt = 1.0 / max(VIEWER_HZ, 1e-3)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            st = time.time()
            mujoco.mj_step(model, data)

            with viewer.lock():
                viewer.user_scn.ngeom = 0
                for _, rgba, pts_mm in paths_mm:
                    pts_world = mm_to_world_xy(pts_mm, minx, maxy, sx, sy)
                    for i in range(len(pts_world)):
                        draw_sphere_point(
                            viewer,
                            float(pts_world[i, 0]),
                            float(pts_world[i, 1]),
                            PATH_Z_M,
                            PATH_POINT_RADIUS_M,
                            rgba,
                        )
                draw_coordinate_overlay(viewer, origin, x_unit, y_unit, minx, maxy, sx, sy)

            viewer.sync()
            rem = dt - (time.time() - st)
            if rem > 0:
                time.sleep(rem)


if __name__ == "__main__":
    main()



