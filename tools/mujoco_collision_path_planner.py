#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 MuJoCo XML 的 collision meshes，自动栅格化代价地图，
并可视化以下路径类型：
1) 直线
2) 圆弧
3) 直线 + 圆弧相切
4) 三次贝塞尔

同时支持 A* 自动规划结果对比。
"""

from __future__ import annotations

import math
import heapq
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np


# =============================================================================
# 你主要改这里
# =============================================================================

# 你的 MuJoCo 场景 XML 路径（可改成绝对路径）
XML_PATH = Path("mjcf/robocon2026.xml")

# 规划区域（左半场，单位 mm）
FIELD_W_MM = 12000.0
FIELD_H_MM = 12000.0
PLAN_X_MIN_MM = 0.0
PLAN_X_MAX_MM = 6000.0
PLAN_Y_MIN_MM = 0.0
PLAN_Y_MAX_MM = 12000.0

# 栅格分辨率与底盘膨胀半径（mm）
GRID_RES_MM = 50.0
ROBOT_RADIUS_MM = 420.0

# A* 起终点（mm）
ASTAR_START_MM = (1200.0, 900.0)
ASTAR_GOAL_MM = (2600.0, 10800.0)
ASTAR_DIAGONAL = True

# 手动路径候选（使用你工程里的路径类型）
PATH_CANDIDATES = [
    {
        "name": "line_arc_tangent_demo",
        "color": "#ef6c00",
        "segments": [
            {
                "type": "tangent_line_arc",
                "line_start": (1200.0, 900.0),
                "arc_end": (2600.0, 7800.0),
                "center": (3200.0, 7800.0),
                "arc_ccw": True,
                "line_samples": 60,
                "arc_samples": 120,
            },
            {
                "type": "bezier",
                "p0": (2600.0, 7800.0),
                "p1": (2400.0, 9000.0),
                "p2": (2400.0, 10000.0),
                "p3": (2600.0, 10800.0),
                "samples": 120,
            },
        ],
    },
]


# =============================================================================
# 数据结构
# =============================================================================

Point = Tuple[float, float]


@dataclass
class MeshData:
    vertices: np.ndarray  # (N,3)
    faces: np.ndarray     # (M,3)


@dataclass
class RasterResult:
    occ: np.ndarray
    res_mm: float
    width: int
    height: int
    x_min_mm: float
    y_min_mm: float


# =============================================================================
# OBJ / XML 读取
# =============================================================================

def parse_vec3(text: Optional[str], default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if not text:
        return np.array(default, dtype=np.float64)
    vals = [float(x) for x in text.strip().split()]
    if len(vals) == 1:
        vals = [vals[0], vals[0], vals[0]]
    if len(vals) != 3:
        raise ValueError(f"vec3 解析失败: {text}")
    return np.array(vals, dtype=np.float64)


def parse_quat(text: Optional[str]) -> np.ndarray:
    # MuJoCo quat: w x y z
    if not text:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    vals = [float(x) for x in text.strip().split()]
    if len(vals) != 4:
        raise ValueError(f"quat 解析失败: {text}")
    return np.array(vals, dtype=np.float64)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def load_obj_mesh(path: Path) -> MeshData:
    verts: List[List[float]] = []
    faces: List[List[int]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                parts = line.split()[1:]
                idx = []
                for p in parts:
                    token = p.split("/")[0]
                    idx.append(int(token) - 1)
                if len(idx) == 3:
                    faces.append(idx)
                elif len(idx) > 3:
                    for i in range(1, len(idx) - 1):
                        faces.append([idx[0], idx[i], idx[i + 1]])
    if not verts or not faces:
        raise ValueError(f"OBJ 无有效网格: {path}")
    return MeshData(vertices=np.array(verts, dtype=np.float64), faces=np.array(faces, dtype=np.int32))


def iter_bodies(body_elem: ET.Element, parent_pos: np.ndarray, parent_rot: np.ndarray):
    pos = parse_vec3(body_elem.get("pos"))
    quat = parse_quat(body_elem.get("quat"))
    rot = quat_to_rotmat(quat)
    world_rot = parent_rot @ rot
    world_pos = parent_pos + parent_rot @ pos
    yield body_elem, world_pos, world_rot
    for child in body_elem.findall("body"):
        yield from iter_bodies(child, world_pos, world_rot)



def _expand_includes_recursive(node: ET.Element, base_dir: Path, stack: Set[Path]) -> None:
    # 递归展开当前节点下所有 <include file="..."/>，避免漏掉外部拆分的场景部件。
    children = list(node)
    for child in children:
        if child.tag == "include":
            include_file = child.get("file")
            if not include_file:
                node.remove(child)
                continue

            include_path = (base_dir / include_file).resolve()
            if include_path in stack:
                raise RuntimeError(f"检测到循环 include: {include_path}")
            if not include_path.exists():
                raise FileNotFoundError(f"include 文件不存在: {include_path}")

            stack.add(include_path)
            inc_tree = ET.parse(include_path)
            inc_root = inc_tree.getroot()
            _expand_includes_recursive(inc_root, include_path.parent, stack)

            # MuJoCo include 文件通常也是 <mujoco> 根；插入其子节点即可。
            if inc_root.tag == "mujoco":
                insert_nodes = list(inc_root)
            else:
                insert_nodes = [inc_root]

            insert_at = list(node).index(child)
            for ins in insert_nodes:
                node.insert(insert_at, ins)
                insert_at += 1
            node.remove(child)
            stack.remove(include_path)
        else:
            _expand_includes_recursive(child, base_dir, stack)


def parse_xml_with_includes(xml_path: Path) -> ET.Element:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    _expand_includes_recursive(root, xml_path.parent.resolve(), {xml_path.resolve()})
    return root
def read_collision_triangles(xml_path: Path) -> np.ndarray:
    if not xml_path.exists():
        raise FileNotFoundError(f"XML 不存在: {xml_path}")

    root = parse_xml_with_includes(xml_path)

    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir") if compiler is not None else ""
    mesh_dir = (xml_path.parent / meshdir).resolve() if meshdir else xml_path.parent.resolve()

    # 资产 mesh 映射
    mesh_map: Dict[str, Path] = {}
    asset = root.find("asset")
    if asset is not None:
        for m in asset.findall("mesh"):
            name = m.get("name")
            file = m.get("file")
            if name and file:
                mesh_map[name] = (mesh_dir / file).resolve()

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("XML 缺少 worldbody")

    # 遍历所有 body 中的 geom；只取 class=scene_collision 的 mesh
    all_tris = []
    cache: Dict[Path, MeshData] = {}
    sc_mesh_geoms = 0
    sc_mesh_in_map = 0
    missing_mesh_names: List[str] = []
    missing_mesh_files: List[str] = []

    for top_body in worldbody.findall("body"):
        for body, bpos, brot in iter_bodies(top_body, np.zeros(3), np.eye(3)):
            for g in body.findall("geom"):
                gclass = g.get("class", "")
                gtype = g.get("type")
                if not gtype and gclass == "scene_collision":
                    # scene_collision 常通过 default 继承 type="mesh"
                    gtype = "mesh"
                if not gtype:
                    gtype = "sphere"
                if gclass != "scene_collision" or gtype != "mesh":
                    continue

                sc_mesh_geoms += 1
                mesh_name = g.get("mesh")
                if not mesh_name:
                    missing_mesh_names.append("<empty>")
                    continue
                if mesh_name not in mesh_map:
                    missing_mesh_names.append(mesh_name)
                    continue

                sc_mesh_in_map += 1
                mesh_path = mesh_map[mesh_name]
                if not mesh_path.exists():
                    missing_mesh_files.append(str(mesh_path))
                    continue
                if mesh_path not in cache:
                    cache[mesh_path] = load_obj_mesh(mesh_path)
                mesh = cache[mesh_path]

                gpos = parse_vec3(g.get("pos"))
                grot = quat_to_rotmat(parse_quat(g.get("quat")))
                world_rot = brot @ grot
                world_pos = bpos + brot @ gpos

                v = (mesh.vertices @ world_rot.T) + world_pos
                tri = v[mesh.faces]  # (M,3,3)
                all_tris.append(tri)

    if not all_tris:
        miss_name_cnt = len(missing_mesh_names)
        miss_file_cnt = len(missing_mesh_files)
        sample_names = ", ".join(sorted(set(missing_mesh_names))[:8]) if miss_name_cnt else "无"
        sample_files = "\n".join(sorted(set(missing_mesh_files))[:8]) if miss_file_cnt else "无"
        raise RuntimeError(
            "未从 XML 提取到 scene_collision mesh 三角面。\n"
            f"xml={xml_path}\nmeshdir={mesh_dir}\n"
            f"scene_collision_mesh_geoms={sc_mesh_geoms}, in_asset_map={sc_mesh_in_map}\n"
            f"missing_mesh_name_count={miss_name_cnt}, sample={sample_names}\n"
            f"missing_mesh_file_count={miss_file_cnt}, sample_paths=\n{sample_files}"
        )
    return np.concatenate(all_tris, axis=0)

# =============================================================================
# 栅格化（自动对齐到 12m 场地）
# =============================================================================

def auto_metric_to_field(tri_xyz: np.ndarray) -> np.ndarray:
    # tri_xyz 单位默认按 MuJoCo(m)。这里自动线性映射到 [0,12000]x[0,12000] mm。
    xy = tri_xyz[:, :, :2].reshape(-1, 2)
    minx, miny = xy[:, 0].min(), xy[:, 1].min()
    maxx, maxy = xy[:, 0].max(), xy[:, 1].max()
    sx = FIELD_W_MM / max(maxx - minx, 1e-9)
    sy = FIELD_H_MM / max(maxy - miny, 1e-9)

    out = tri_xyz.copy()
    out[:, :, 0] = (out[:, :, 0] - minx) * sx
    # y 轴翻转到“向下为正”地图坐标
    out[:, :, 1] = (maxy - out[:, :, 1]) * sy
    return out


def mm_to_grid(p: Point, raster: RasterResult) -> Tuple[int, int]:
    gx = int(round((p[0] - raster.x_min_mm) / raster.res_mm))
    gy = int(round((p[1] - raster.y_min_mm) / raster.res_mm))
    return gx, gy


def rasterize_triangles(tri_mm: np.ndarray, res_mm: float, robot_r_mm: float) -> RasterResult:
    width = int(round((PLAN_X_MAX_MM - PLAN_X_MIN_MM) / res_mm)) + 1
    height = int(round((PLAN_Y_MAX_MM - PLAN_Y_MIN_MM) / res_mm)) + 1
    occ = np.zeros((height, width), dtype=np.uint8)

    for tri in tri_mm:
        pts_mm = tri[:, :2]
        pts_px = []
        for p in pts_mm:
            gx, gy = mm_to_grid((float(p[0]), float(p[1])), RasterResult(occ, res_mm, width, height, PLAN_X_MIN_MM, PLAN_Y_MIN_MM))
            pts_px.append([gx, gy])
        poly = np.array(pts_px, dtype=np.int32)
        if poly.shape == (3, 2):
            cv2.fillConvexPoly(occ, poly, 255)

    # 对障碍进行机器人半径膨胀
    inflate_px = int(math.ceil(robot_r_mm / res_mm))
    if inflate_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflate_px * 2 + 1, inflate_px * 2 + 1))
        occ = cv2.dilate(occ, k, iterations=1)

    return RasterResult(occ=occ, res_mm=res_mm, width=width, height=height, x_min_mm=PLAN_X_MIN_MM, y_min_mm=PLAN_Y_MIN_MM)


# =============================================================================
# 路径段生成（工程同款语义）
# =============================================================================

def gen_line(start: Point, end: Point, samples: int = 80) -> np.ndarray:
    t = np.linspace(0.0, 1.0, max(2, samples))
    x = start[0] + (end[0] - start[0]) * t
    y = start[1] + (end[1] - start[1]) * t
    return np.column_stack([x, y])


def wrap_2pi(a: float) -> float:
    while a < 0.0:
        a += 2.0 * math.pi
    while a >= 2.0 * math.pi:
        a -= 2.0 * math.pi
    return a


def gen_arc(center: Point, radius: float, start_angle: float, end_angle: float, ccw: bool, samples: int = 120) -> np.ndarray:
    a0 = wrap_2pi(start_angle)
    a1 = wrap_2pi(end_angle)
    if ccw:
        sweep = a1 - a0
        if sweep < 0:
            sweep += 2 * math.pi
        ang = np.linspace(a0, a0 + sweep, max(2, samples))
    else:
        sweep = a0 - a1
        if sweep < 0:
            sweep += 2 * math.pi
        ang = np.linspace(a0, a0 - sweep, max(2, samples))
    x = center[0] + radius * np.cos(ang)
    y = center[1] + radius * np.sin(ang)
    return np.column_stack([x, y])


def select_tangent_point(line_start: Point, arc_end: Point, center: Point, arc_ccw: bool) -> Optional[Tuple[Point, float]]:
    cx, cy = center
    radius = math.hypot(arc_end[0] - cx, arc_end[1] - cy)
    vx = line_start[0] - cx
    vy = line_start[1] - cy
    d2 = vx * vx + vy * vy
    r2 = radius * radius
    if radius < 1e-6 or d2 <= r2 + 1e-6:
        return None

    base = r2 / d2
    factor = radius * math.sqrt(d2 - r2) / d2
    perp = (-vy, vx)
    candidates = [
        (cx + base * vx + factor * perp[0], cy + base * vy + factor * perp[1]),
        (cx + base * vx - factor * perp[0], cy + base * vy - factor * perp[1]),
    ]

    def arc_angle(start: Point, end: Point) -> float:
        sa = math.atan2(start[1] - cy, start[0] - cx)
        ea = math.atan2(end[1] - cy, end[0] - cx)
        ccw_delta = wrap_2pi(ea - sa)
        if arc_ccw:
            return ccw_delta
        return (2 * math.pi if ccw_delta < 1e-6 else (2 * math.pi - ccw_delta))

    for p in candidates:
        line_dir = np.array([p[0] - line_start[0], p[1] - line_start[1]], dtype=float)
        rdir = np.array([p[0] - cx, p[1] - cy], dtype=float)
        tan = np.array([-rdir[1], rdir[0]]) if arc_ccw else np.array([rdir[1], -rdir[0]])
        nl = np.linalg.norm(line_dir)
        nt = np.linalg.norm(tan)
        if nl < 1e-6 or nt < 1e-6:
            continue
        line_dir /= nl
        tan /= nt
        if float(np.dot(line_dir, tan)) > 0.99:
            return (p, arc_angle(p, arc_end))
    return None


def gen_tangent_line_arc(line_start: Point, arc_end: Point, center: Point, arc_ccw: bool, line_samples: int, arc_samples: int) -> np.ndarray:
    sel = select_tangent_point(line_start, arc_end, center, arc_ccw)
    if sel is None:
        return np.zeros((0, 2), dtype=float)
    tangent, _ = sel
    line_pts = gen_line(line_start, tangent, line_samples)
    r = math.hypot(tangent[0] - center[0], tangent[1] - center[1])
    a0 = math.atan2(tangent[1] - center[1], tangent[0] - center[0])
    a1 = math.atan2(arc_end[1] - center[1], arc_end[0] - center[0])
    arc_pts = gen_arc(center, r, a0, a1, arc_ccw, arc_samples)
    return np.vstack([line_pts, arc_pts[1:]])


def gen_bezier(p0: Point, p1: Point, p2: Point, p3: Point, samples: int = 140) -> np.ndarray:
    t = np.linspace(0.0, 1.0, max(2, samples))
    u = 1.0 - t
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    p3 = np.array(p3, dtype=float)
    pts = (
        (u ** 3)[:, None] * p0[None, :]
        + (3.0 * (u ** 2) * t)[:, None] * p1[None, :]
        + (3.0 * u * (t ** 2))[:, None] * p2[None, :]
        + (t ** 3)[:, None] * p3[None, :]
    )
    return pts


def build_candidate_points(segments: Sequence[dict]) -> np.ndarray:
    all_pts: List[np.ndarray] = []
    for seg in segments:
        tp = seg.get("type")
        if tp == "line":
            pts = gen_line(seg["start"], seg["end"], seg.get("samples", 80))
        elif tp == "arc":
            pts = gen_arc(seg["center"], seg["radius"], seg["start_angle"], seg["end_angle"], seg["ccw"], seg.get("samples", 120))
        elif tp == "tangent_line_arc":
            pts = gen_tangent_line_arc(seg["line_start"], seg["arc_end"], seg["center"], seg["arc_ccw"], seg.get("line_samples", 60), seg.get("arc_samples", 120))
        elif tp == "bezier":
            pts = gen_bezier(seg["p0"], seg["p1"], seg["p2"], seg["p3"], seg.get("samples", 140))
        else:
            raise ValueError(f"未知路径段类型: {tp}")
        if len(pts) == 0:
            continue
        if all_pts:
            all_pts.append(pts[1:])
        else:
            all_pts.append(pts)
    return np.vstack(all_pts) if all_pts else np.zeros((0, 2), dtype=float)


# =============================================================================
# A* 自动路径
# =============================================================================

def astar_on_occ(raster: RasterResult, start_mm: Point, goal_mm: Point, diagonal: bool = True) -> np.ndarray:
    occ_bin = (raster.occ > 0).astype(np.uint8)
    h, w = occ_bin.shape

    s = mm_to_grid(start_mm, raster)
    g = mm_to_grid(goal_mm, raster)
    sx, sy = s
    gx, gy = g
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return np.zeros((0, 2), dtype=float)
    if occ_bin[sy, sx] or occ_bin[gy, gx]:
        return np.zeros((0, 2), dtype=float)

    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if diagonal:
        nbrs += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

    def heur(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    pq = [(0.0, (sx, sy))]
    g_cost = {(sx, sy): 0.0}
    came = {}

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == (gx, gy):
            break
        for dx, dy in nbrs:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if occ_bin[ny, nx]:
                continue
            ng = g_cost[cur] + math.hypot(dx, dy)
            nxt = (nx, ny)
            if ng < g_cost.get(nxt, 1e30):
                g_cost[nxt] = ng
                came[nxt] = cur
                heapq.heappush(pq, (ng + heur(nxt, (gx, gy)), nxt))

    if (gx, gy) not in came and (sx, sy) != (gx, gy):
        return np.zeros((0, 2), dtype=float)

    path = [(gx, gy)]
    while path[-1] != (sx, sy):
        path.append(came[path[-1]])
    path.reverse()
    mm = np.array([(p[0] * raster.res_mm + raster.x_min_mm, p[1] * raster.res_mm + raster.y_min_mm) for p in path], dtype=float)
    return mm


# =============================================================================
# 评估与可视化
# =============================================================================

def eval_path(points: np.ndarray, raster: RasterResult) -> Tuple[float, float]:
    if len(points) < 2:
        return 0.0, 1.0
    d = np.diff(points, axis=0)
    length = float(np.sum(np.hypot(d[:, 0], d[:, 1])))
    occ = raster.occ
    hit = 0
    total = 0
    for p in points:
        gx, gy = mm_to_grid((float(p[0]), float(p[1])), raster)
        if 0 <= gx < raster.width and 0 <= gy < raster.height:
            total += 1
            if occ[gy, gx] > 0:
                hit += 1
    collision_ratio = hit / max(total, 1)
    return length, collision_ratio


def visualize(raster: RasterResult, astar_pts: np.ndarray, candidate_pts: List[Tuple[str, str, np.ndarray]]) -> None:
    plt.figure(figsize=(11, 9))
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    occ_show = (raster.occ > 0).astype(np.uint8)
    extent = [PLAN_X_MIN_MM, PLAN_X_MAX_MM, PLAN_Y_MAX_MM, PLAN_Y_MIN_MM]
    plt.imshow(occ_show, cmap="Reds", alpha=0.45, extent=extent, interpolation="nearest")

    if len(astar_pts) > 1:
        plt.plot(astar_pts[:, 0], astar_pts[:, 1], "-", lw=2.5, color="#1565c0", label="A* 自动路径")

    for name, color, pts in candidate_pts:
        if len(pts) > 1:
            plt.plot(pts[:, 0], pts[:, 1], "-", lw=2.2, color=color, label=name)

    plt.scatter([ASTAR_START_MM[0]], [ASTAR_START_MM[1]], c="#00c853", s=70, label="起点")
    plt.scatter([ASTAR_GOAL_MM[0]], [ASTAR_GOAL_MM[1]], c="#d50000", s=90, marker="*", label="终点")

    plt.xlim(PLAN_X_MIN_MM - 100, PLAN_X_MAX_MM + 100)
    plt.ylim(PLAN_Y_MAX_MM + 100, PLAN_Y_MIN_MM - 100)
    plt.gca().set_aspect("equal")
    plt.grid(True, linestyle="--", alpha=0.35)
    plt.title("MuJoCo collision mesh 栅格代价地图 + 路径可视化")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.legend(loc="upper right", fontsize=9)
    plt.tight_layout()
    plt.show()


def resolve_xml_path() -> Path:
    # 若配置路径不存在，自动尝试工程里常见的 MuJoCo 场景入口。
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
            if p != XML_PATH:
                print(f"[info] XML_PATH 不存在，自动回退到: {p}")
            return p
    raise FileNotFoundError(
        "找不到可用场景 XML。请检查 XML_PATH 或确认 mjcf 目录下有 robocon2026.xml / scene_*.xml"
    )

def main() -> None:
    xml_path = resolve_xml_path()
    tri_xyz = read_collision_triangles(xml_path)
    tri_mm = auto_metric_to_field(tri_xyz)
    raster = rasterize_triangles(tri_mm, GRID_RES_MM, ROBOT_RADIUS_MM)

    astar_pts = astar_on_occ(raster, ASTAR_START_MM, ASTAR_GOAL_MM, diagonal=ASTAR_DIAGONAL)
    if len(astar_pts) == 0:
        print("[warn] A* 未找到可行路径，请调整起终点/膨胀半径/栅格分辨率")
    else:
        l, c = eval_path(astar_pts, raster)
        print(f"[info] A*: length={l:.1f}mm, collision_ratio={c:.3f}, points={len(astar_pts)}")

    candidate_pts: List[Tuple[str, str, np.ndarray]] = []
    for candi in PATH_CANDIDATES:
        name = candi["name"]
        color = candi.get("color", "#ef6c00")
        pts = build_candidate_points(candi["segments"])
        if len(pts) == 0:
            print(f"[warn] 候选路径 {name} 为空")
            continue
        l, c = eval_path(pts, raster)
        print(f"[info] {name}: length={l:.1f}mm, collision_ratio={c:.3f}, points={len(pts)}")
        candidate_pts.append((name, color, pts))

    visualize(raster, astar_pts, candidate_pts)


if __name__ == "__main__":
    main()











