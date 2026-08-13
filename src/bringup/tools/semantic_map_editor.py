#!/usr/bin/env python3
"""语义地图 msgpack 标注工具 —— 隧道版。

Lucifer 的 rm_map_server 只读 msgpack，这个 GUI 就是它的唯一编辑入口：改 terrain
（补围栏、删噪点、抹掉/画出障碍）、把通道格标成 TUNNEL 并画出轴线、给每条隧道填
净高/净宽/提前量/限速窗。取代 HW 的 terrain_label_editor.py，语义换成隧道。

与 HW 台阶版的根本差别：
  * 隧道方向是无向轴线。画一条线，线的走向就是轴线，正反等价（C++ 侧用 |cos θ|），
    所以箭头画成两头的杆而不是单向箭头。台阶的方向是「上行」，有正反。
  * 多了 origin（地图原点非零，见 GridGeometry）、tunnels[] 规格表、tunnel_ids
    逐格通道。保存前按 semantic_map.cpp 的 loadSemanticMap 同一套不变量自检，
    坏图直接拒绝写出 —— 宁可在这里炸，也不要让车开进墙里。

不依赖 open3d：读 PCD 复用同目录的 pcd_reader（纯 numpy）。

用法：
    python3 semantic_map_editor.py [地图.msgpack]
"""

from __future__ import annotations

import math
import sys
import tkinter as tk
from collections.abc import Iterable
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional

import msgpack
import numpy as np
from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    # PCD 解析（read_pcd_xyz）住在同目录的 pcd_reader 里。
    import pcd_reader
    _HAS_PCD = True
except ImportError:
    _HAS_PCD = False

# 必须与 semantic_map.hpp 的 TerrainType 一致。
TERRAIN_FLAT = 0
TERRAIN_OBSTACLE = 1
TERRAIN_TUNNEL = 2
TERRAIN_UNKNOWN = 7

# 工具栏可选的标签：(值, 名字, 画布配色)。只列真正会用到的四种；3~6 是
# semantic_map.hpp 里给坡/台阶预留的，隧道场景用不上，故意不放进来避免误标。
PALETTE = [
    (TERRAIN_FLAT, "FLAT 空地", (235, 235, 235)),
    (TERRAIN_OBSTACLE, "OBSTACLE 障碍", (40, 40, 40)),
    (TERRAIN_TUNNEL, "TUNNEL 隧道", (70, 130, 220)),
    (TERRAIN_UNKNOWN, "UNKNOWN 未知", (150, 150, 150)),
]
LABEL_COLOR = {value: color for value, _, color in PALETTE}
# 隧道格叠一层随 id 变化的描边色，方便区分同图里的多条隧道。
TUNNEL_ID_TINT = [
    (70, 130, 220), (220, 120, 60), (110, 190, 90),
    (190, 90, 190), (90, 190, 190), (200, 180, 60),
]

MAX_UNDO = 30
TWO_PI = 2.0 * math.pi

# 隧道规格字段：(键, 界面标签, 默认值)。顺序即界面里输入框的排布顺序。
# 与 semantic_map.hpp 的 TunnelSpec 一一对应。
TUNNEL_FIELDS = [
    ("clear_height", "净高 clear_height (m)", 0.30),
    ("clear_width", "净宽 clear_width (m)", 0.50),
    ("run_up", "提前量 run_up (m)", 1.20),
    ("velocity_min", "限速下限 velocity_min (m/s)", 0.0),
    ("velocity_max", "限速上限 velocity_max (m/s)", 0.0),
]


def encode_axis(angle_rad: float) -> int:
    """轴线角 → 0~255 编码，与 semantic_map.cpp 的 decodeAxis 互为逆。

    隧道无向：angle 与 angle+π 是同一条轴。这里不在编码阶段折叠到 [0, π)，
    留给 C++ 侧的 |cos θ| 处理，保证 encode/decode 严格可逆、round-trip 不掉信息。
    """
    return int(round(angle_rad % TWO_PI / TWO_PI * 255.0)) % 256


def decode_axis(encoded: int) -> float:
    """0~255 编码 → 轴线角（弧度），与 encode_axis 互逆。"""
    return encoded / 255.0 * TWO_PI


class ValidationError(Exception):
    """保存前自检失败。信息直接照搬 C++ loadSemanticMap 的措辞，方便对照。"""


class SemanticMapModel:
    """一张语义地图的内存表示，逐格可改，保存前按 C++ 不变量自检。

    terrain / direction / tunnel_id 三个通道都是 (height, width) 的 numpy 数组，
    行主序，index = y*width + x —— 和 msgpack 的字节布局、GridGeometry.index 完全一致。
    y=0 是地图坐标系 y 最小的一行（不是图像顶端）；画布显示时才上下翻转。
    """

    def __init__(self, width: int, height: int, resolution: float,
                 origin: tuple[float, float]) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = (float(origin[0]), float(origin[1]))
        self.terrain = np.full((height, width), TERRAIN_FLAT, dtype=np.uint8)
        self.direction = np.zeros((height, width), dtype=np.uint8)
        # 0 表示不属于任何隧道；隧道格存 id+1，与 msgpack 的 tunnel_ids 约定一致。
        self.tunnel_id = np.zeros((height, width), dtype=np.uint8)
        self.tunnels: list[dict] = []
        self.path: Optional[Path] = None

    # ---- 载入 ----
    @classmethod
    def load(cls, path: Path) -> "SemanticMapModel":
        raw = msgpack.unpackb(path.read_bytes(), raw=False)
        if not isinstance(raw, dict):
            raise ValidationError(f"{path}: msgpack 根不是 map")
        width = int(raw["width"])
        height = int(raw["height"])
        resolution = float(raw["resolution"])
        origin_field = raw.get("origin", [0.0, 0.0, 0.0])
        origin = (float(origin_field[0]), float(origin_field[1]))
        model = cls(width, height, resolution, origin)

        cells = width * height
        terrain = np.frombuffer(bytes(raw["terrain"]), dtype=np.uint8)
        if terrain.size != cells:
            raise ValidationError(
                f"terrain 有 {terrain.size} 格，但栅格是 {cells}")
        model.terrain = terrain.reshape((height, width)).copy()

        direction = np.frombuffer(bytes(raw["direction"]), dtype=np.uint8)
        if direction.size != cells:
            raise ValidationError(
                f"direction 有 {direction.size} 格，但栅格是 {cells}")
        model.direction = direction.reshape((height, width)).copy()

        ids_field = raw.get("tunnel_ids")
        if ids_field is not None:
            ids = np.frombuffer(bytes(ids_field), dtype=np.uint8)
            if ids.size != cells:
                raise ValidationError(
                    f"tunnel_ids 有 {ids.size} 格，但栅格是 {cells}")
            model.tunnel_id = ids.reshape((height, width)).copy()

        for entry in raw.get("tunnels", []):
            spec = {key: float(entry.get(key, default))
                    for key, _, default in TUNNEL_FIELDS}
            model.tunnels.append(spec)
        model.path = path
        return model

    # ---- 保存 ----
    def validate(self) -> None:
        """按 semantic_map.cpp::loadSemanticMap 的同一套不变量自检。

        坏图宁可在这里炸，也不要写出去让 C++ 加载时才拒绝 —— 那时错误信息离
        用户的操作已经很远了。
        """
        labels = np.unique(self.terrain)
        bad = labels[labels >= 8]
        if bad.size:
            raise ValidationError(f"terrain 出现越界标签 {list(bad)}（必须 < 8）")

        directional = self.terrain == TERRAIN_TUNNEL
        # 非方向标签不得带方向：否则膨胀时会被当成轴线源传播。
        stray = (~directional) & (self.direction != 0)
        if np.any(stray):
            ys, xs = np.nonzero(stray)
            raise ValidationError(
                f"格 ({xs[0]},{ys[0]}) 是 {terrain_label_name(int(self.terrain[ys[0], xs[0]]))} "
                f"却带了方向 {int(self.direction[ys[0], xs[0]])}；非隧道格方向必须为 0")

        tunnel_cells = int(np.count_nonzero(directional))
        if tunnel_cells > 0 and not self.tunnels:
            raise ValidationError(
                "有隧道格却没有任何隧道规格；C++ 侧需要 clear_height 判断底盘能否通过")

        # 每个隧道格的 id 必须落在 [1, len(tunnels)]。
        if tunnel_cells > 0:
            ids_here = self.tunnel_id[directional]
            n = len(self.tunnels)
            bad_ids = ids_here[(ids_here < 1) | (ids_here > n)]
            if bad_ids.size:
                raise ValidationError(
                    f"有隧道格的 tunnel_id={int(bad_ids[0])} 越界；已声明 {n} 条隧道，"
                    "每个隧道格必须先归属到某条隧道")

        # 非隧道格不该残留 id，否则语义含糊。这个 C++ 侧不查，但保存前顺手清掉。
        for index, spec in enumerate(self.tunnels):
            label = f"tunnels[{index}]"
            if not (spec["clear_height"] > 0.0) or not (spec["clear_width"] > 0.0):
                raise ValidationError(f"{label} 的净高和净宽必须为正")
            if (spec["run_up"] < 0.0 or spec["velocity_min"] < 0.0 or
                    spec["velocity_max"] < 0.0):
                raise ValidationError(f"{label} 的 run_up/限速不能为负")
            if spec["velocity_max"] > 0.0 and spec["velocity_min"] > spec["velocity_max"]:
                raise ValidationError(f"{label} 的限速下限高于上限")

    def to_payload(self) -> dict:
        self.validate()
        # 非隧道格清掉残留方向与 id，保证写出的通道干净。
        direction = self.direction.copy()
        tunnel_id = self.tunnel_id.copy()
        non_tunnel = self.terrain != TERRAIN_TUNNEL
        direction[non_tunnel] = 0
        tunnel_id[non_tunnel] = 0

        payload = {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": [self.origin[0], self.origin[1], 0.0],
            "terrain": self.terrain.astype(np.uint8).tobytes(),
            "direction": direction.astype(np.uint8).tobytes(),
            "tunnels": [
                {key: spec[key] for key, _, _ in TUNNEL_FIELDS}
                for spec in self.tunnels
            ],
        }
        # 只有真存在多条隧道、需要区分时才写 tunnel_ids；否则省掉，C++ 侧空 ids
        # 表示所有隧道格共用 tunnels[0]。
        if np.any(tunnel_id != 0):
            payload["tunnel_ids"] = tunnel_id.astype(np.uint8).tobytes()
        return payload

    def save(self, path: Path) -> None:
        payload = self.to_payload()
        path.write_bytes(msgpack.packb(payload, use_bin_type=True))
        self.path = path


def terrain_label_name(label: int) -> str:
    for value, name, _ in PALETTE:
        if value == label:
            return name.split()[0]
    return "reserved"


class EditorApp:
    """tkinter 主界面。

    坐标约定：模型数组 y=0 是地图坐标系 y 最小的一行；画布上「上」是 y 大的方向，
    所以显示时纵向翻转（disp_row = height-1-y）。所有鼠标坐标都先过 _canvas_to_cell
    转回模型 (x, y)，界面里再没有第二处翻转。
    """

    def __init__(self, root: tk.Tk, model: SemanticMapModel) -> None:
        self.root = root
        self.model = model
        self.zoom = self._fit_zoom()
        self.active_label = TERRAIN_OBSTACLE
        self.mode = "pixel"          # pixel | rect | line
        self.brush = 1
        self.active_tunnel = 0        # 当前隧道在 self.model.tunnels 里的下标
        self.undo_stack: list[tuple] = []
        self.drag_start: Optional[tuple[int, int]] = None
        self.pcd_overlay: Optional[np.ndarray] = None   # (h, w) 归一化高度或 None
        self.show_overlay = tk.BooleanVar(value=False)
        self.show_arrows = tk.BooleanVar(value=True)
        self._photo: Optional[ImageTk.PhotoImage] = None

        root.title(self._title())
        self._build_toolbar()
        self._build_body()
        self._build_statusbar()
        self._bind_keys()
        self._sync_tunnel_panel()
        self._render()

    def _title(self) -> str:
        name = self.model.path.name if self.model.path else "未命名"
        return f"语义地图编辑器 — {name}"

    def _fit_zoom(self) -> int:
        # 让整张图初始时大致铺满一个 1000px 窗口，限制在 1~12 之间。
        span = max(self.model.width, self.model.height)
        return max(1, min(12, 1000 // max(1, span)))

    def _build_toolbar(self) -> None:
        bar = ttk.Frame(self.root, padding=4)
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")

        ttk.Label(bar, text="标签:").pack(side=tk.LEFT)
        self._label_var = tk.IntVar(value=self.active_label)
        for value, name, _ in PALETTE:
            ttk.Radiobutton(bar, text=name, value=value, variable=self._label_var,
                            command=self._on_label_change).pack(side=tk.LEFT)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Label(bar, text="工具:").pack(side=tk.LEFT)
        self._mode_var = tk.StringVar(value=self.mode)
        for value, name in (("pixel", "点/刷"), ("rect", "矩形"), ("line", "隧道轴线")):
            ttk.Radiobutton(bar, text=name, value=value, variable=self._mode_var,
                            command=self._on_mode_change).pack(side=tk.LEFT)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Label(bar, text="笔刷:").pack(side=tk.LEFT)
        self._brush_var = tk.IntVar(value=self.brush)
        ttk.Spinbox(bar, from_=1, to=50, width=3, textvariable=self._brush_var,
                    command=self._on_brush_change).pack(side=tk.LEFT)
        ttk.Label(bar, text="格 (隧道宽度会自动同步笔刷)", foreground="#666").pack(
            side=tk.LEFT, padx=(2, 0))

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(bar, text="放大", command=lambda: self._set_zoom(self.zoom + 1)).pack(side=tk.LEFT)
        ttk.Button(bar, text="缩小", command=lambda: self._set_zoom(self.zoom - 1)).pack(side=tk.LEFT)
        ttk.Checkbutton(bar, text="轴线箭头", variable=self.show_arrows,
                        command=self._render).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Checkbutton(bar, text="PCD高度底图", variable=self.show_overlay,
                        command=self._render).pack(side=tk.LEFT)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        ttk.Button(bar, text="载入PCD", command=self._load_pcd).pack(side=tk.LEFT)
        ttk.Button(bar, text="打开", command=self._open_map).pack(side=tk.LEFT)
        ttk.Button(bar, text="保存", command=self._save).pack(side=tk.LEFT)
        ttk.Button(bar, text="另存为", command=self._save_as).pack(side=tk.LEFT)

    def _build_body(self) -> None:
        # 左：画布（带滚动条）；右：隧道规格面板。
        canvas_frame = ttk.Frame(self.root)
        canvas_frame.grid(row=1, column=0, sticky="nsew")
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, background="#202020",
                                highlightthickness=0)
        hbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL,
                             command=self.canvas.xview)
        vbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL,
                             command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>", self._on_hover)

        self._build_tunnel_panel()

    def _build_tunnel_panel(self) -> None:
        panel = ttk.Frame(self.root, padding=6)
        panel.grid(row=1, column=1, sticky="ns")
        ttk.Label(panel, text="隧道规格", font=("", 10, "bold")).pack(anchor="w")

        self._tunnel_list = tk.Listbox(panel, height=6, exportselection=False)
        self._tunnel_list.pack(fill=tk.X, pady=4)
        self._tunnel_list.bind("<<ListboxSelect>>", self._on_tunnel_select)

        btns = ttk.Frame(panel)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="新增", command=self._add_tunnel).pack(side=tk.LEFT)
        ttk.Button(btns, text="删除", command=self._delete_tunnel).pack(side=tk.LEFT)

        self._spec_vars: dict[str, tk.StringVar] = {}
        form = ttk.Frame(panel)
        form.pack(fill=tk.X, pady=6)
        for row, (key, label, _) in enumerate(TUNNEL_FIELDS):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w")
            var = tk.StringVar()
            var.trace_add("write", lambda *_, k=key: self._on_spec_edit(k))
            ttk.Entry(form, textvariable=var, width=8).grid(row=row, column=1, padx=4)
            self._spec_vars[key] = var

        ttk.Label(panel, wraplength=200, foreground="#666", text=(
            "画「隧道轴线」时，当前选中的隧道 id 会写进沿线的格子。轴线无向，"
            "正反等价。净高/净宽必须为正，否则保存会被拒绝。")).pack(
            anchor="w", pady=(8, 0))

    def _build_statusbar(self) -> None:
        self.status = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status, relief=tk.SUNKEN,
                  anchor="w", padding=2).grid(row=2, column=0, columnspan=2, sticky="ew")

    def _bind_keys(self) -> None:
        self.root.bind("<Control-z>", lambda _e: self._undo())
        self.root.bind("<Control-s>", lambda _e: self._save())
        self.root.bind("1", lambda _e: self._pick_label(TERRAIN_FLAT))
        self.root.bind("2", lambda _e: self._pick_label(TERRAIN_OBSTACLE))
        self.root.bind("3", lambda _e: self._pick_label(TERRAIN_TUNNEL))
        self.root.bind("4", lambda _e: self._pick_label(TERRAIN_UNKNOWN))
        self.root.bind("<plus>", lambda _e: self._set_zoom(self.zoom + 1))
        self.root.bind("<minus>", lambda _e: self._set_zoom(self.zoom - 1))

    # ---- 渲染 ----
    def _base_rgb(self) -> np.ndarray:
        """把 terrain 数组渲染成 (h, w, 3) 的 RGB 底图（尚未翻转、未放大）。"""
        h, w = self.model.terrain.shape
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        for value, color in LABEL_COLOR.items():
            rgb[self.model.terrain == value] = color
        # 隧道格按 id 换描边色调，区分同图多条隧道。
        tunnel_mask = self.model.terrain == TERRAIN_TUNNEL
        if np.any(tunnel_mask):
            ids = self.model.tunnel_id
            for idx in range(len(self.model.tunnels)):
                tint = TUNNEL_ID_TINT[idx % len(TUNNEL_ID_TINT)]
                rgb[tunnel_mask & (ids == idx + 1)] = tint
        return rgb

    def _blend_overlay(self, rgb: np.ndarray) -> np.ndarray:
        if not self.show_overlay.get() or self.pcd_overlay is None:
            return rgb
        # 高度灰度 → 与底图 50/50 混合，只为核对语义标注和真实结构是否对得上。
        gray = (self.pcd_overlay * 255.0).astype(np.uint8)
        gray_rgb = np.stack([gray, gray, gray], axis=-1)
        return ((rgb.astype(np.uint16) + gray_rgb.astype(np.uint16)) // 2).astype(np.uint8)

    def _render(self) -> None:
        rgb = self._blend_overlay(self._base_rgb())
        # 模型 y=0 在下，画布 y=0 在上 —— 纵向翻转一次，之后不再翻。
        flipped = np.flipud(rgb)
        image = Image.fromarray(flipped, "RGB")
        if self.zoom != 1:
            image = image.resize((self.model.width * self.zoom,
                                  self.model.height * self.zoom), Image.NEAREST)
        self._photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self._photo)
        self.canvas.configure(scrollregion=(0, 0, image.width, image.height))
        if self.show_arrows.get():
            self._draw_axes()

    def _draw_axes(self) -> None:
        """给每条隧道画一根贯穿其格子的双头轴杆（无向，两端对称）。"""
        tunnel_mask = self.model.terrain == TERRAIN_TUNNEL
        if not np.any(tunnel_mask):
            return
        z = self.zoom
        for idx in range(max(1, len(self.model.tunnels))):
            sel = tunnel_mask & (self.model.tunnel_id == idx + 1)
            if len(self.model.tunnels) == 0:
                sel = tunnel_mask
            if not np.any(sel):
                continue
            ys, xs = np.nonzero(sel)
            cx, cy = xs.mean(), ys.mean()
            # 该隧道所有格的方向编码取众数作代表轴（同一条隧道方向应当一致）。
            angles = self.model.direction[sel]
            enc = int(np.bincount(angles).argmax())
            theta = decode_axis(enc)
            length = 0.5 * math.hypot(xs.ptp() + 1, ys.ptp() + 1)
            dx, dy = math.cos(theta) * length, math.sin(theta) * length
            # 模型→画布：x 不变，y 翻转（disp_y = height-1-y），格心 +0.5。
            top = self.model.height - 1
            x0, y0 = (cx - dx + 0.5) * z, (top - (cy - dy) + 0.5) * z
            x1, y1 = (cx + dx + 0.5) * z, (top - (cy + dy) + 0.5) * z
            self.canvas.create_line(x0, y0, x1, y1, fill="#ffef60", width=2)
            # 两端各画个小圆点，强调无向。
            for ex, ey in ((x0, y0), (x1, y1)):
                self.canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                                        fill="#ffef60", outline="")

    # ---- 坐标换算 ----
    def _canvas_to_cell(self, ex: float, ey: float) -> Optional[tuple[int, int]]:
        cx = self.canvas.canvasx(ex)
        cy = self.canvas.canvasy(ey)
        gx = int(cx // self.zoom)
        disp_y = int(cy // self.zoom)
        gy = self.model.height - 1 - disp_y     # 翻回模型坐标
        if 0 <= gx < self.model.width and 0 <= gy < self.model.height:
            return gx, gy
        return None

    # ---- 编辑操作 ----
    def _snapshot(self) -> None:
        """入栈一份撤销点。三通道一起存，撤销时整体还原。"""
        self.undo_stack.append((
            self.model.terrain.copy(),
            self.model.direction.copy(),
            self.model.tunnel_id.copy(),
        ))
        if len(self.undo_stack) > MAX_UNDO:
            self.undo_stack.pop(0)

    def _undo(self) -> None:
        if not self.undo_stack:
            self.status.set("没有可撤销的操作")
            return
        self.model.terrain, self.model.direction, self.model.tunnel_id = \
            self.undo_stack.pop()
        self._render()
        self.status.set("已撤销")

    def _paint_cells(self, xs: Iterable[int], ys: Iterable[int]) -> None:
        """把一批格子设成当前标签。隧道格顺带写 id 与轴向；非隧道格清掉两者。"""
        xs = np.asarray(list(xs))
        ys = np.asarray(list(ys))
        self.model.terrain[ys, xs] = self.active_label
        if self.active_label == TERRAIN_TUNNEL:
            self.model.tunnel_id[ys, xs] = self.active_tunnel + 1
            # 单点/矩形涂隧道时方向暂设 0，靠「隧道轴线」工具补；这里不覆盖已有轴向。
        else:
            self.model.direction[ys, xs] = 0
            self.model.tunnel_id[ys, xs] = 0

    def _apply_pixel(self, cell: tuple[int, int]) -> None:
        gx, gy = cell
        r = self.brush - 1
        xs, ys = [], []
        for yy in range(max(0, gy - r), min(self.model.height, gy + r + 1)):
            for xx in range(max(0, gx - r), min(self.model.width, gx + r + 1)):
                xs.append(xx)
                ys.append(yy)
        self._paint_cells(xs, ys)

    def _apply_rect(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        x0, x1 = sorted((a[0], b[0]))
        y0, y1 = sorted((a[1], b[1]))
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        self._paint_cells(xs.ravel(), ys.ravel())

    def _apply_line(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        """画隧道轴线：沿 a→b 的格子标成 TUNNEL，方向 = 该线段走向（无向）。

        线的粗细用当前笔刷，方便一次刷出有宽度的通道。方向对所有覆盖格取同一个
        编码值 —— 同一条隧道方向一致，膨胀时才不会互相抵消。
        """
        if self.active_label != TERRAIN_TUNNEL:
            # 轴线工具只对隧道有意义；其余标签退化成画条线。
            pass
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
        enc = encode_axis(theta)
        cells = self._bresenham(a, b)
        r = self.brush - 1

        # 只垂直线段方向扩散,避免笔刷大时轴线"冒出头"。
        # 线段方向 = (cos θ, sin θ), 垂直方向 = (-sin θ, cos θ)
        perp_x, perp_y = -math.sin(theta), math.cos(theta)

        xs, ys = [], []
        for cx, cy in cells:
            # 沿垂直方向 ±r 步扩散,不沿线段方向扩。
            for step in range(-r, r + 1):
                xx = int(round(cx + step * perp_x))
                yy = int(round(cy + step * perp_y))
                if 0 <= xx < self.model.width and 0 <= yy < self.model.height:
                    xs.append(xx)
                    ys.append(yy)

        xs_a = np.asarray(xs)
        ys_a = np.asarray(ys)
        self.model.terrain[ys_a, xs_a] = self.active_label
        if self.active_label == TERRAIN_TUNNEL:
            self.model.direction[ys_a, xs_a] = enc
            self.model.tunnel_id[ys_a, xs_a] = self.active_tunnel + 1
        else:
            self.model.direction[ys_a, xs_a] = 0
            self.model.tunnel_id[ys_a, xs_a] = 0

    @staticmethod
    def _bresenham(a: tuple[int, int], b: tuple[int, int]) -> list[tuple[int, int]]:
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        cells = []
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return cells

    def _on_press(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            return
        if self.mode == "pixel":
            self._snapshot()
            self._apply_pixel(cell)
            self._render()
        else:
            # 矩形 / 轴线：按下记起点，松开时一次成型。
            self.drag_start = cell

    def _on_drag(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            return
        if self.mode == "pixel":
            self._apply_pixel(cell)
            self._render()
        self._update_hover(cell)

    def _on_release(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if self.mode == "pixel":
            self.drag_start = None
            return
        if cell is None or self.drag_start is None:
            self.drag_start = None
            return
        self._snapshot()
        if self.mode == "rect":
            self._apply_rect(self.drag_start, cell)
        elif self.mode == "line":
            self._apply_line(self.drag_start, cell)
        self.drag_start = None
        self._render()

    def _on_hover(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is not None:
            self._update_hover(cell)

    def _update_hover(self, cell: tuple[int, int]) -> None:
        gx, gy = cell
        wx = self.model.origin[0] + (gx + 0.5) * self.model.resolution
        wy = self.model.origin[1] + (gy + 0.5) * self.model.resolution
        label = terrain_label_name(int(self.model.terrain[gy, gx]))
        extra = ""
        if self.model.terrain[gy, gx] == TERRAIN_TUNNEL:
            tid = int(self.model.tunnel_id[gy, gx])
            ang = math.degrees(decode_axis(int(self.model.direction[gy, gx])))
            extra = f"  tunnel_id={tid}  轴向={ang:.0f}°"
        self.status.set(
            f"格({gx},{gy})  世界({wx:.2f},{wy:.2f})  {label}{extra}")

    # ---- 工具栏回调 ----
    def _on_label_change(self) -> None:
        self.active_label = self._label_var.get()
        if self.active_label == TERRAIN_TUNNEL and not self.model.tunnels:
            # 一标隧道就得有规格可挂，否则保存必失败。自动建一条默认隧道。
            self._add_tunnel()

    def _pick_label(self, value: int) -> None:
        self._label_var.set(value)
        self._on_label_change()

    def _on_mode_change(self) -> None:
        self.mode = self._mode_var.get()
        if self.mode == "line":
            # 轴线工具默认配合隧道标签使用。
            self._pick_label(TERRAIN_TUNNEL)

    def _on_brush_change(self) -> None:
        self.brush = max(1, int(self._brush_var.get()))

    def _set_zoom(self, zoom: int) -> None:
        self.zoom = max(1, min(20, zoom))
        self._render()

    # ---- 隧道面板 ----
    def _sync_tunnel_panel(self) -> None:
        self._tunnel_list.delete(0, tk.END)
        for i, spec in enumerate(self.model.tunnels):
            self._tunnel_list.insert(
                tk.END,
                f"#{i + 1}  h={spec['clear_height']:.2f} w={spec['clear_width']:.2f}")
        if self.model.tunnels:
            self.active_tunnel = min(self.active_tunnel, len(self.model.tunnels) - 1)
            self._tunnel_list.selection_clear(0, tk.END)
            self._tunnel_list.selection_set(self.active_tunnel)
            self._load_spec_into_form(self.active_tunnel)
        else:
            self.active_tunnel = 0
            for var in self._spec_vars.values():
                var.set("")

    def _load_spec_into_form(self, index: int) -> None:
        spec = self.model.tunnels[index]
        for key, var in self._spec_vars.items():
            # 屏蔽 trace 回写：set 会触发 _on_spec_edit，这里只是刷新显示。
            self._loading_form = True
            var.set(f"{spec[key]:g}")
            self._loading_form = False

    def _on_tunnel_select(self, _event: tk.Event) -> None:
        sel = self._tunnel_list.curselection()
        if sel:
            self.active_tunnel = sel[0]
            self._load_spec_into_form(self.active_tunnel)
            # 切换隧道时同步笔刷到该隧道的宽度，方便接着画轴线。
            w = self.model.tunnels[self.active_tunnel].get("clear_width", 0.0)
            if w > 0:
                brush = max(1, int(round(w / self.model.resolution)))
                self.brush = brush
                self._brush_var.set(brush)

    def _on_spec_edit(self, key: str) -> None:
        if getattr(self, "_loading_form", False):
            return
        if not self.model.tunnels or self.active_tunnel >= len(self.model.tunnels):
            return
        text = self._spec_vars[key].get().strip()
        if text == "":
            return
        try:
            value = float(text)
            self.model.tunnels[self.active_tunnel][key] = value
            # 编辑隧道净宽时，自动同步笔刷 = 净宽对应的格数，方便画轴线。
            if key == "clear_width" and value > 0:
                brush = max(1, int(round(value / self.model.resolution)))
                self.brush = brush
                self._brush_var.set(brush)
                self.status.set(f"笔刷已同步到 {brush} 格 ({value:.2f}m)")
        except ValueError:
            self.status.set(f"{key} 需要是数字")

    def _add_tunnel(self) -> None:
        self.model.tunnels.append(
            {key: default for key, _, default in TUNNEL_FIELDS})
        self.active_tunnel = len(self.model.tunnels) - 1
        self._sync_tunnel_panel()
        # 新增隧道时同步笔刷到默认宽度，方便立即画轴线。
        w = self.model.tunnels[self.active_tunnel]["clear_width"]
        if w > 0:
            brush = max(1, int(round(w / self.model.resolution)))
            self.brush = brush
            self._brush_var.set(brush)
        self.status.set(f"新增隧道 #{self.active_tunnel + 1}，笔刷已同步到 {self.brush} 格")

    def _delete_tunnel(self) -> None:
        if not self.model.tunnels:
            return
        index = self.active_tunnel
        used = np.count_nonzero(self.model.tunnel_id == index + 1)
        if used and not messagebox.askyesno(
                "删除隧道", f"隧道 #{index + 1} 还有 {used} 个格子在用，删了会把它们"
                "留成没有归属的隧道格（保存会被拒绝）。仍要删除？"):
            return
        self._snapshot()
        del self.model.tunnels[index]
        # 重排后续 id：> index+1 的减一，== index+1 的清零。
        ids = self.model.tunnel_id
        ids[ids == index + 1] = 0
        ids[ids > index + 1] -= 1
        self.active_tunnel = max(0, index - 1)
        self._sync_tunnel_panel()
        self._render()

    # ---- PCD 高度底图 ----
    def _load_pcd(self) -> None:
        if not _HAS_PCD:
            messagebox.showerror("载入PCD", "找不到 pcd_reader.py，无法解析 PCD")
            return
        path = filedialog.askopenfilename(
            title="选择 PCD", filetypes=[("PCD", "*.pcd"), ("所有文件", "*")])
        if not path:
            return
        try:
            xyz = pcd_reader.read_pcd_xyz(path)
        except (OSError, ValueError) as error:
            messagebox.showerror("载入PCD", f"读取失败: {error}")
            return
        self.pcd_overlay = self._grid_max_height(xyz)
        self.show_overlay.set(True)
        self._render()
        self.status.set(f"载入 {len(xyz)} 点做高度底图")

    def _grid_max_height(self, xyz: np.ndarray) -> np.ndarray:
        """把点云按当前地图的格网投影成逐格最大高度，归一化到 0~1。

        用最大高度而不是平均：核对隧道时最关心「这一格头顶有没有东西」，最高点
        最能体现顶板和横梁。格网与模型完全对齐（同 origin / resolution / 尺寸），
        y 轴不翻转 —— 翻转只发生在渲染阶段。
        """
        h, w = self.model.height, self.model.width
        ox, oy = self.model.origin
        res = self.model.resolution
        gx = np.floor((xyz[:, 0] - ox) / res).astype(int)
        gy = np.floor((xyz[:, 1] - oy) / res).astype(int)
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        gx, gy, gz = gx[inside], gy[inside], xyz[inside, 2]
        grid = np.full((h, w), np.nan, dtype=np.float32)
        # 逐点取最大：np.maximum.at 累积到重复索引上。
        flat = gy * w + gx
        acc = np.full(h * w, -np.inf, dtype=np.float32)
        np.maximum.at(acc, flat, gz)
        acc[acc == -np.inf] = np.nan
        grid = acc.reshape((h, w))
        finite = np.isfinite(grid)
        if not np.any(finite):
            return np.zeros((h, w), dtype=np.float32)
        lo = np.nanmin(grid)
        hi = np.nanmax(grid)
        span = hi - lo if hi > lo else 1.0
        norm = np.where(finite, (grid - lo) / span, 0.0)
        return norm.astype(np.float32)

    # ---- 文件 ----
    def _open_map(self) -> None:
        path = filedialog.askopenfilename(
            title="打开语义地图", filetypes=[("msgpack", "*.msgpack"), ("所有文件", "*")])
        if not path:
            return
        try:
            self.model = SemanticMapModel.load(Path(path))
        except (OSError, ValueError, KeyError, ValidationError) as error:
            messagebox.showerror("打开失败", str(error))
            return
        self.undo_stack.clear()
        self.pcd_overlay = None
        self.zoom = self._fit_zoom()
        self.root.title(self._title())
        self._sync_tunnel_panel()
        self._render()
        self.status.set(f"已打开 {path}")

    def _save(self) -> None:
        if self.model.path is None:
            self._save_as()
            return
        self._write(self.model.path)

    def _save_as(self) -> None:
        path = filedialog.asksaveasfilename(
            title="另存为", defaultextension=".msgpack",
            filetypes=[("msgpack", "*.msgpack")])
        if path:
            self._write(Path(path))

    def _write(self, path: Path) -> None:
        try:
            self.model.save(path)
        except ValidationError as error:
            messagebox.showerror("保存被拒绝（自检未过）", str(error))
            self.status.set(f"保存失败: {error}")
            return
        except OSError as error:
            messagebox.showerror("保存失败", str(error))
            return
        self.root.title(self._title())
        self.status.set(f"已保存 {path}")


def main(argv: Optional[list[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    root = tk.Tk()
    if argv:
        try:
            model = SemanticMapModel.load(Path(argv[0]))
        except (OSError, ValueError, KeyError, ValidationError) as error:
            print(f"打开失败: {error}", file=sys.stderr)
            return 1
    else:
        # 没给文件时开一张空白图，让用户先 打开 或 另存为。尺寸随手给个小默认。
        model = SemanticMapModel(width=100, height=100, resolution=0.05,
                                 origin=(0.0, 0.0))
    EditorApp(root, model)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
