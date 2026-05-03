# tools 目录说明

本目录存放比赛场地建模、路径规划、MuJoCo 场景碰撞地图分析，以及若干路径曲线可视化工具。脚本默认从仓库根目录运行，坐标单位除特别说明外均为 `mm`。

## 环境依赖

基础可视化脚本需要：

```powershell
pip install numpy matplotlib
```

使用 MuJoCo 碰撞地图、OpenCV 栅格化或 3D viewer 时还需要：

```powershell
pip install opencv-python mujoco
```

使用九宫格交互规划器时需要：

```powershell
pip install pygame
```

建议在仓库根目录运行脚本，例如：

```powershell
cd E:\_Document\r2_ws\r2_chassis
python tools\mujoco_map_planner.py
```

## 文件概览

| 文件 | 作用 | 主要依赖 |
| --- | --- | --- |
| `map_path_manual_planner.py` | 按规则尺寸绘制 R2 左半场，并叠加手动配置的 line / arc / bezier 路径 | `numpy`, `matplotlib` |
| `mujoco_map_planner.py` | 使用规则尺寸的简化场地模型生成占据栅格，执行 A* 路径规划，并优先用 MuJoCo viewer 显示，缺少 MuJoCo 时回退到 2D 图 | `numpy`, `matplotlib`, 可选 `mujoco` |
| `mujoco_collision_path_planner.py` | 读取 MuJoCo XML 中 `class=scene_collision` 的 mesh，自动栅格化碰撞地图，对比 A* 路径和手动候选路径 | `numpy`, `matplotlib`, `opencv-python` |
| `mujoco_viewer_path_overlay.py` | 在真实 MuJoCo 场景中叠加路径点、A* 路径和局部坐标系辅助显示 | `numpy`, `mujoco`, 并复用 `mujoco_collision_path_planner.py` |
| `visualize_bezier_curve.py` | 可视化三次贝塞尔曲线、控制点、切向箭头和曲率热度 | `numpy`, `matplotlib` |
| `visualize_tangent_line_circle.py` | 可视化直线与圆弧相切路径，计算切点并绘制线段和圆弧 | `matplotlib` |
| `plan1.py` | 基于 `pygame` 的 2D 九宫格/节点交互规划器，用于摆放 R1、R2、假目标并计算可行路径 | `pygame` |
| `robocon_field_atlas_v1.pdf` | 场地资料/图册参考文件 | PDF 阅读器 |

## 脚本使用说明

### 1. `map_path_manual_planner.py`

功能：

- 绘制 12000 x 12000 mm 场地中的 R2 左半场。
- 标注一区、二区、三区、启动区、梅林、坡道、九宫格等区域。
- 按 `PATH_SEGMENTS` 绘制手动路径，支持 `line`、`arc`、`bezier`。
- 统计路径总长度，以及路径采样点中可达/越界/禁区数量。

运行：

```powershell
python tools\map_path_manual_planner.py
```

常改参数在文件顶部：

- `START_POINT`：路径起点。
- `END_POINT`：路径终点。
- `END_YAW_RAD`：终点朝向，单位为弧度。
- `PATH_SEGMENTS`：手动路径段列表。
- `LINE_SAMPLES`、`ARC_SAMPLES`、`BEZIER_SAMPLES`：不同路径段的采样密度。

路径段示例：

```python
PATH_SEGMENTS = [
    {"type": "line", "start": (1200.0, 900.0), "end": (2300.0, 2100.0)},
    {
        "type": "bezier",
        "p0": (2300.0, 2100.0),
        "p1": (2200.0, 4200.0),
        "p2": (2100.0, 6800.0),
        "p3": (2300.0, 8200.0),
    },
]
```

### 2. `mujoco_map_planner.py`

功能：

- 使用规则尺寸构造简化场地模型。
- 将可达区域和禁行区域转换成占据栅格。
- 使用 A* 从 `START` 规划到 `GOAL`。
- 对路径做简单平滑。
- 如果安装了 `mujoco`，会打开 MuJoCo viewer；否则使用 `matplotlib` 显示 2D 地图。

运行：

```powershell
python tools\mujoco_map_planner.py
```

常改参数：

- `START` / `GOAL`：A* 起终点。
- `GRID_RES`：栅格分辨率，数值越小越精细但越慢。
- `ROBOT_RADIUS`：机器人半径，用于障碍物膨胀。
- `USE_DIAGONAL`：是否允许 8 邻接搜索。
- `SMOOTH_ITERS`：路径平滑迭代次数，`0` 表示不平滑。

### 3. `mujoco_collision_path_planner.py`

功能：

- 读取 `mjcf/robocon2026.xml` 或其他候选 MuJoCo 场景 XML。
- 解析 `<include>` 文件。
- 提取 `class=scene_collision` 且类型为 mesh 的碰撞几何。
- 将 mesh 投影到场地平面，按 `GRID_RES_MM` 栅格化，并按 `ROBOT_RADIUS_MM` 膨胀。
- 运行 A*，同时评估 `PATH_CANDIDATES` 中的手动候选路径长度和碰撞比例。
- 用 2D 图显示碰撞地图、A* 路径和候选路径。

运行：

```powershell
python tools\mujoco_collision_path_planner.py
```

常改参数：

- `XML_PATH`：MuJoCo 场景 XML 路径，默认 `mjcf/robocon2026.xml`。
- `PLAN_X_MIN_MM` / `PLAN_X_MAX_MM` / `PLAN_Y_MIN_MM` / `PLAN_Y_MAX_MM`：规划区域。
- `GRID_RES_MM`：栅格分辨率。
- `ROBOT_RADIUS_MM`：机器人碰撞膨胀半径。
- `ASTAR_START_MM` / `ASTAR_GOAL_MM`：A* 起终点。
- `PATH_CANDIDATES`：候选路径列表。

支持的候选路径段类型：

- `line`：直线段，需要 `start`、`end`。
- `arc`：圆弧段，需要 `center`、`radius`、`start_angle`、`end_angle`、`ccw`。
- `tangent_line_arc`：直线加相切圆弧，需要 `line_start`、`arc_end`、`center`、`arc_ccw`。
- `bezier`：三次贝塞尔曲线，需要 `p0`、`p1`、`p2`、`p3`。

### 4. `mujoco_viewer_path_overlay.py`

功能：

- 打开真实 MuJoCo 场景。
- 复用 `mujoco_collision_path_planner.py` 的碰撞读取、A* 和候选路径采样逻辑。
- 将路径点叠加到 MuJoCo viewer 中。
- 支持局部坐标系输入：以 R2 启动区参考点作为原点，用 R2 指向 R1 的方向定义局部 `+X`。
- 可绘制局部坐标轴和半场分界线辅助检查坐标。

运行：

```powershell
python tools\mujoco_viewer_path_overlay.py
```

常改参数：

- `XML_PATH`：场景 XML。
- `R2_START_GLOBAL_MM` / `R1_START_GLOBAL_MM`：用于定义局部坐标系的两个全场坐标点。
- `AXIS_HANDEDNESS`：坐标系方向，取 `"left"` 或 `"right"`。
- `INPUT_USE_LOCAL_FRAME`：为 `True` 时，`PATH_CANDIDATES` 和 A* 起终点按局部坐标解释；为 `False` 时按全场坐标解释。
- `SHOW_ASTAR`：是否叠加 A* 路径。
- `PATH_CANDIDATES`：要叠加显示的手动路径。

### 5. `visualize_bezier_curve.py`

功能：

- 根据四个控制点绘制三次贝塞尔曲线。
- 显示控制多边形、控制点、切向方向箭头。
- 用颜色表示归一化曲率。
- 可选保存图片。

运行示例：

```powershell
python tools\visualize_bezier_curve.py --p0 0 0 --p1 800 0 --p2 1200 1200 --p3 2000 1000
```

保存图片：

```powershell
python tools\visualize_bezier_curve.py --p0 0 0 --p1 800 0 --p2 1200 1200 --p3 2000 1000 --save tools\bezier_demo.png
```

参数：

- `--p0`：起点。
- `--p1` / `--p2`：控制点。
- `--p3`：终点。
- `--samples`：采样点数，默认 `400`。
- `--tangent-count`：切向箭头数量，默认 `12`。
- `--save`：保存路径，不填则弹出窗口显示。

### 6. `visualize_tangent_line_circle.py`

功能：

- 输入直线起点、圆弧终点、圆心和圆弧方向。
- 自动计算直线与圆的切点。
- 绘制 `line_start -> tangent_point` 的直线段，以及 `tangent_point -> arc_end` 的圆弧段。

运行示例：

```powershell
python tools\visualize_tangent_line_circle.py --line-start 0 0 --arc-end 1000 800 --center 1200 1200 --arc-ccw 1
```

保存图片：

```powershell
python tools\visualize_tangent_line_circle.py --line-start 0 0 --arc-end 1000 800 --center 1200 1200 --arc-ccw 1 --save tools\tangent_arc_demo.png
```

参数：

- `--line-start`：直线起点。
- `--arc-end`：圆弧终点。
- `--center`：圆心。
- `--arc-ccw`：圆弧方向，`1` 表示逆时针，`0` 表示顺时针。
- `--save`：保存路径，不填则弹出窗口显示。

### 7. `plan1.py`

功能：

- 打开一个 `pygame` 窗口，显示 12 个网格节点、入口节点和出口节点。
- 通过右侧按钮选择放置类型：空、R1、R2、Fake。
- 点击左侧节点放置或擦除目标。
- 点击 `START PLANNING` 后根据高度、相邻关系、抓取/移除代价等规则计算路径。
- 在窗口中显示移动路径、抓取动作、移除动作和总代价。

运行：

```powershell
python tools\plan1.py
```

交互规则概要：

- R1 最多放置 4 个。
- R2 最多放置 4 个。
- Fake 最多放置 1 个。
- Fake 不能放在入口侧节点 `0`、`1`、`2`。
- R1 不能放在节点 `4`、`7`。
- 至少需要 2 个 R2 节点才能规划。
- 相邻节点高度差必须为 `200` 才能通行。

## 修改路径的建议流程

1. 先用 `visualize_bezier_curve.py` 或 `visualize_tangent_line_circle.py` 单独检查曲线形状。
2. 再把确认后的路径参数写入 `map_path_manual_planner.py` 或 `mujoco_collision_path_planner.py` 的 `PATH_SEGMENTS` / `PATH_CANDIDATES`。
3. 用 `mujoco_collision_path_planner.py` 检查路径长度和碰撞比例。
4. 需要在 3D 场景中确认位置时，再运行 `mujoco_viewer_path_overlay.py`。

## 注意事项

- 这些脚本大多没有命令行配置入口，主要通过修改文件顶部的常量来调整参数。
- MuJoCo 相关脚本依赖 `mjcf`、`meshes` 目录中的场景和网格文件，建议始终从仓库根目录运行。
- 如果中文标签显示为方块或乱码，请安装中文字体，或修改脚本中的 `plt.rcParams["font.sans-serif"]` 字体列表。
- `tools/__pycache__` 是 Python 自动生成缓存目录，不属于工具源码。
