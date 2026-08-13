#!/usr/bin/env python3
"""把 Super-LIO 建好的 PCD 点云直接转成 rm_map_server 读的语义地图 msgpack。

注意：这条链只适合地面平整的点云。全局高度切片对地面起伏大的场地会满图误判
（实测 RMUL.pcd 地面起伏 ~0.4 m，任何全局 z 阈值都同时切在某些区域的地面上和
另一些区域的墙下面）。主链已换成 slam_toolbox 建图：mode:=mapping 起建图链
（见 real/sim launch），map_saver_cli 存 pgm+yaml，再用 pgm_to_navmap.py 转
msgpack。本脚本保留作为平整场地/快速验证的备用路径。

terrain 三态直接来自点云高度切片（见下方 build_grid）：
  * 障碍点足够多的格子 → OBSTACLE(1)
  * 只被地面点覆盖的格子 → FLAT(0)
  * 完全没点的格子      → UNKNOWN(7)
UNKNOWN 必须独立保留、不能塌成 FLAT：从未扫到的区域要吃 A* 的 unknown_cost 惩罚，
否则规划器会以正常代价穿过没观测过的地方（见 semantic_map.hpp 的 TerrainType）。

隧道格这一步一律留空（terrain 里没有 TUNNEL、direction 全零、tunnels 为空）。
隧道是「能站但要摆姿态才能进」的先验，点云区分不出，必须人工在编辑器里标。

只依赖 numpy + msgpack。用法：
    python3 pcd_to_navmap.py RMUL.pcd -o ../map/RMUL.msgpack
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import msgpack
import numpy as np

# 必须与 semantic_map.hpp 的 TerrainType 一致。
TERRAIN_FLAT = 0
TERRAIN_OBSTACLE = 1
TERRAIN_UNKNOWN = 7

# 占据栅格的三态中间值。沿用 rm_map_server 的灰度约定，只作为本脚本内部
# build_grid -> grid_to_terrain 的中转标记，不再写出任何 pgm。
PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205
PIXEL_FREE = 254

_PCD_TYPE_MAP = {
    ('F', 4): 'f4', ('F', 8): 'f8',
    ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('U', 8): 'u8',
    ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4', ('I', 8): 'i8',
}


def _read_pcd_header(stream):
    """逐行读 PCD 头，返回 (字段字典, 数据段起始偏移)。"""
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PCD 头在遇到 DATA 之前就结束了')
        text = line.decode('ascii', errors='replace').strip()
        if not text or text.startswith('#'):
            continue
        key, _, value = text.partition(' ')
        header[key.upper()] = value.strip()
        if key.upper() == 'DATA':
            return header, stream.tell()


def read_pcd_xyz(path):
    """读 PCD 的 xyz，返回 (N, 3) float64。支持 ascii / binary。

    binary_compressed 用的是 LZF，标准库没有解码器，这里直接报错并给出
    转换命令，而不是悄悄返回错的点。
    """
    with open(path, 'rb') as stream:
        header, data_offset = _read_pcd_header(stream)

        fields = header.get('FIELDS', '').split()
        sizes = [int(v) for v in header.get('SIZE', '').split()]
        types = header.get('TYPE', '').split()
        counts = [int(v) for v in header.get('COUNT', '').split()] or [1] * len(fields)
        data_kind = header.get('DATA', '').lower()
        points = int(header.get('POINTS') or
                     int(header.get('WIDTH', 0)) * int(header.get('HEIGHT', 1)))

        if not fields or not all(axis in fields for axis in ('x', 'y', 'z')):
            raise ValueError(f'PCD 缺少 x/y/z 字段，实际字段为 {fields}')
        if len(sizes) != len(fields) or len(types) != len(fields):
            raise ValueError('PCD 头里 FIELDS/SIZE/TYPE 数量不一致')
        if any(c != 1 for c in counts):
            raise ValueError('不支持 COUNT>1 的 PCD 字段')

        if data_kind == 'ascii':
            stream.seek(data_offset)
            table = np.loadtxt(stream, dtype=np.float64, usecols=range(len(fields)),
                               ndmin=2)
            idx = [fields.index(a) for a in ('x', 'y', 'z')]
            return table[:, idx]

        if data_kind == 'binary_compressed':
            raise ValueError(
                'binary_compressed PCD 需要 LZF 解码，标准库不支持。先转成 binary：\n'
                '  pcl_convert_pcd_ascii_binary in.pcd out.pcd 1')

        if data_kind != 'binary':
            raise ValueError(f'不认识的 PCD DATA 类型: {data_kind!r}')

        dtype = np.dtype([
            (name, _PCD_TYPE_MAP[(t.upper(), s)])
            for name, t, s in zip(fields, types, sizes)
        ])
        stream.seek(data_offset)
        raw = np.frombuffer(stream.read(points * dtype.itemsize), dtype=dtype,
                            count=points)
        return np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)


def build_grid(xyz, resolution, z_min, z_max, ground_z_max, min_points, padding):
    """把点云投影成占据栅格。

    分三类：z 落在 [z_min, z_max] 且落点数 >= min_points 的格子算障碍；只被
    地面点（z <= ground_z_max）覆盖过的格子算空闲；完全没点的格子算未知。
    只用地面点标空闲很关键——否则墙后面从未被扫到的区域会被误标成可通行。

    返回 (grid, origin_x, origin_y)。grid[0] 是 y 最小的一行。
    """
    if xyz.size == 0:
        raise ValueError('点云是空的')

    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if xyz.size == 0:
        raise ValueError('点云里没有有限值的点')

    obstacle = xyz[(xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)]
    ground = xyz[xyz[:, 2] <= ground_z_max]

    # 用全部点定边界，保证空闲区域也在图内。
    min_xy = xyz[:, :2].min(axis=0) - padding
    max_xy = xyz[:, :2].max(axis=0) + padding
    size = np.ceil((max_xy - min_xy) / resolution).astype(int) + 1
    width, height = int(size[0]), int(size[1])
    if width <= 0 or height <= 0:
        raise ValueError('算出来的栅格尺寸非法，检查 resolution')

    def to_cells(points):
        cells = np.floor((points[:, :2] - min_xy) / resolution).astype(np.int64)
        np.clip(cells[:, 0], 0, width - 1, out=cells[:, 0])
        np.clip(cells[:, 1], 0, height - 1, out=cells[:, 1])
        return cells[:, 1] * width + cells[:, 0]

    flat = height * width
    obstacle_hits = np.bincount(to_cells(obstacle), minlength=flat) if obstacle.size else \
        np.zeros(flat, dtype=np.int64)
    ground_hits = np.bincount(to_cells(ground), minlength=flat) if ground.size else \
        np.zeros(flat, dtype=np.int64)

    grid = np.full(flat, PIXEL_UNKNOWN, dtype=np.uint8)
    grid[ground_hits > 0] = PIXEL_FREE
    grid[obstacle_hits >= min_points] = PIXEL_OCCUPIED

    # 常见坑：喂进来的 PCD 是已经拍平过的薄片（比如给 2D GICP 用的那种），
    # z 跨度不足以区分地面和障碍。这种情况下高度切片几乎筛不出东西，
    # 结果是一张几乎全空闲的图，直接拿去导航会撞墙。宁可吵一句。
    z_span = float(xyz[:, 2].max() - xyz[:, 2].min())
    if z_span < (z_max - z_min):
        print(f'警告: 点云 z 跨度只有 {z_span:.2f} m，小于障碍切片厚度 '
              f'{z_max - z_min:.2f} m。这份 PCD 大概是拍平过的，'
              '按高度分离地面/障碍不成立。', file=sys.stderr)
    if obstacle.size and (obstacle_hits >= min_points).sum() * 200 < (ground_hits > 0).sum():
        print('警告: 判为占据的格子远少于空闲格子，--z-min/--z-max 很可能没对上'
              '这份点云的高度区间。先看下面打印的统计，再决定是否调整。',
              file=sys.stderr)

    return grid.reshape(height, width), float(min_xy[0]), float(min_xy[1])


def grid_to_terrain(grid: np.ndarray) -> np.ndarray:
    """把占据栅格灰度映射成 terrain 标签数组。

    grid[0] 是 y 最小的一行（build_grid 的约定），正是 msgpack terrain 的
    行主序布局 index = y*width + x，所以不需要翻转 —— pgm 那条链之所以要翻，
    是因为 pgm 第一行是图像顶端，这里没有 pgm 就没有那次翻转。
    """
    terrain = np.full(grid.shape, TERRAIN_UNKNOWN, dtype=np.uint8)
    terrain[grid == PIXEL_FREE] = TERRAIN_FLAT
    terrain[grid == PIXEL_OCCUPIED] = TERRAIN_OBSTACLE
    return terrain


def convert(xyz: np.ndarray, args) -> tuple[dict, dict]:
    grid, origin_x, origin_y = build_grid(
        xyz, args.resolution, args.z_min, args.z_max, args.ground_z_max,
        args.min_points, args.padding)
    terrain = grid_to_terrain(grid)
    height, width = terrain.shape

    payload = {
        "width": width,
        "height": height,
        "resolution": args.resolution,
        "origin": [origin_x, origin_y, 0.0],
        # C-order tobytes(): index = y*width + x，与 map_server_node.cpp 一致。
        "terrain": terrain.tobytes(),
        # 没有隧道格，方向通道全零。C++ 侧会校验「非方向标签不得带方向」。
        "direction": bytes(width * height),
        "tunnels": [],
    }
    info = {
        "width": width,
        "height": height,
        "origin": (origin_x, origin_y),
        "occupied": int((terrain == TERRAIN_OBSTACLE).sum()),
        "free": int((terrain == TERRAIN_FLAT).sum()),
        "unknown": int((terrain == TERRAIN_UNKNOWN).sum()),
    }
    return payload, info


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="把 PCD 点云直接转成 rm_map_server 用的语义地图 msgpack",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("pcd", help="输入 PCD 文件")
    parser.add_argument(
        "-o", "--output", default=None,
        help="输出 msgpack 路径，默认与 PCD 同名同目录（换成 .msgpack 后缀）")
    parser.add_argument("-f", "--force", action="store_true",
                        help="覆盖已存在的输出文件")
    parser.add_argument("-r", "--resolution", type=float, default=0.05,
                        help="栅格分辨率（米/格），需与其它场地图保持一致")
    parser.add_argument("--z-min", type=float, default=0.15,
                        help="障碍物切片下界（米，地图坐标系）")
    parser.add_argument("--z-max", type=float, default=1.20,
                        help="障碍物切片上界（米）。设在车高附近，避开顶棚和横梁")
    parser.add_argument("--ground-z-max", type=float, default=0.10,
                        help="低于此高度的点视为地面，用来标记空闲区域")
    parser.add_argument("--min-points", type=int, default=3,
                        help="格子内障碍点数达到此值才判占据，用于抑制孤立噪点")
    parser.add_argument("--padding", type=float, default=0.5,
                        help="地图四周留白（米）")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.z_min > args.z_max:
        print("错误: --z-min 不能大于 --z-max", file=sys.stderr)
        return 2
    if args.resolution <= 0.0:
        print("错误: --resolution 必须为正", file=sys.stderr)
        return 2

    output = Path(args.output) if args.output else Path(args.pcd).with_suffix(".msgpack")
    if output.exists() and not args.force:
        print(f"输出已存在，用 -f 覆盖: {output}", file=sys.stderr)
        return 1

    try:
        xyz = read_pcd_xyz(args.pcd)
        payload, info = convert(xyz, args)
    except (OSError, ValueError) as error:
        print(f"转换失败: {error}", file=sys.stderr)
        return 1

    output.write_bytes(msgpack.packb(payload, use_bin_type=True))

    total = info["width"] * info["height"]
    print(f"读入 {len(xyz)} 点 -> {info['width']}x{info['height']} 栅格 "
          f"@ {args.resolution} m/格")
    print(f"  origin=({info['origin'][0]:.3f}, {info['origin'][1]:.3f})")
    print(f"  obstacle {info['occupied']:8d} ({100.0 * info['occupied'] / total:5.2f}%)")
    print(f"  flat     {info['free']:8d} ({100.0 * info['free'] / total:5.2f}%)")
    print(f"  unknown  {info['unknown']:8d} ({100.0 * info['unknown'] / total:5.2f}%)")
    print("  tunnel          0 (点云区分不出，用 semantic_map_editor.py 人工标)")
    print(f"  写出 {output}")
    print("提示: 生成后建议用 semantic_map_editor.py 核对，补上激光扫不到的围栏、"
          "删掉观众席噪点，并标注隧道。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
