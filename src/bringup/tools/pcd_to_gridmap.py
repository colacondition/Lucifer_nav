#!/usr/bin/env python3
"""把 Super-LIO 建好的 PCD 点云转成 rm_map_server 能读的 .pgm + .yaml。

为什么需要这个脚本：Lucifer 没有在线建图链，`/map` 由 rm_map_server 直接读
`map/<world>.pgm` 得到，所以 HL 那套 `map_saver_cli` 的做法在这里是循环的
（存的就是刚读进来的图）。换新场地时唯一的输入是 PCD，必须离线转一次。

输出严格对齐 rm_map_server 的读图约定（见 navigation2/src/map_server_node.cpp）：
  * P5 二进制 PGM，第 0 行是图像顶部，对应栅格 y 最大的一行；
  * negate=0，占据度 = 1 - pixel/max_value，因此黑(0)=占据、白(254)=空闲；
  * 205 落在 free_thresh..occupied_thresh 之间，被读成未知(-1)。

只依赖 numpy。用法：
    python3 pcd_to_gridmap.py RMUL.pcd -o ../map/RMUL
"""

import argparse
import os
import re
import sys

import numpy as np

# 与 rm_map_server 默认阈值配套的灰度值：占据/未知/空闲。
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


def write_pgm(path, grid):
    """写 P5 PGM。grid[0] 是 y 最小的一行，而 PGM 第一行是图像顶部，
    所以要上下翻转——rm_map_server 读的时候会再翻回来。"""
    height, width = grid.shape
    with open(path, 'wb') as out:
        out.write(f'P5\n# Generated by pcd_to_gridmap.py\n{width} {height}\n255\n'
                  .encode('ascii'))
        out.write(np.flipud(grid).tobytes())


# free_thresh 必须 <= 1 - PIXEL_UNKNOWN/255 == 0.196，否则 rm_map_server 会把
# 未知格子(205)判成空闲，全局规划器就敢用正常代价穿过从未扫到的区域。
# 这也是 nav2 默认取 0.196 而不是 0.25 的原因。
FREE_THRESH = 0.196
OCCUPIED_THRESH = 0.65


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    with open(path, 'w') as out:
        out.write(
            f'image: {image_name}\n'
            'mode: trinary\n'
            f'resolution: {resolution:g}\n'
            f'origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n'
            'negate: 0\n'
            f'occupied_thresh: {OCCUPIED_THRESH:g}\n'
            f'free_thresh: {FREE_THRESH:g}\n'
            'map_start_pose: [0.000, 0.000, 0.000]\n')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='把 PCD 点云转成 rm_map_server 用的 .pgm + .yaml',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('pcd', help='输入 PCD 文件')
    parser.add_argument(
        '-o', '--output', required=True,
        help='输出前缀，不带扩展名。例如 src/bringup/map/RMUL')
    parser.add_argument('-r', '--resolution', type=float, default=0.05,
                        help='栅格分辨率（米/格），需与其它场地图保持一致')
    parser.add_argument('--z-min', type=float, default=0.15,
                        help='障碍物切片下界（米，地图坐标系）')
    parser.add_argument('--z-max', type=float, default=1.20,
                        help='障碍物切片上界（米）。设在车高附近，避开顶棚和横梁')
    parser.add_argument('--ground-z-max', type=float, default=0.10,
                        help='低于此高度的点视为地面，用来标记空闲区域')
    parser.add_argument('--min-points', type=int, default=3,
                        help='格子内障碍点数达到此值才判占据，用于抑制孤立噪点')
    parser.add_argument('--padding', type=float, default=0.5,
                        help='地图四周留白（米）')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.z_min > args.z_max:
        print('错误: --z-min 不能大于 --z-max', file=sys.stderr)
        return 2
    if args.resolution <= 0.0:
        print('错误: --resolution 必须为正', file=sys.stderr)
        return 2

    xyz = read_pcd_xyz(args.pcd)
    grid, origin_x, origin_y = build_grid(
        xyz, args.resolution, args.z_min, args.z_max, args.ground_z_max,
        args.min_points, args.padding)

    prefix = os.path.splitext(args.output)[0]
    pgm_path = prefix + '.pgm'
    yaml_path = prefix + '.yaml'
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, os.path.basename(pgm_path), args.resolution,
               origin_x, origin_y)

    height, width = grid.shape
    occupied = int((grid == PIXEL_OCCUPIED).sum())
    free = int((grid == PIXEL_FREE).sum())
    unknown = grid.size - occupied - free
    print(f'读入 {len(xyz)} 点 -> {width}x{height} 栅格 @ {args.resolution} m/格')
    print(f'  origin=({origin_x:.3f}, {origin_y:.3f})  '
          f'占据 {occupied} / 空闲 {free} / 未知 {unknown}')
    print(f'  写出 {pgm_path}')
    print(f'  写出 {yaml_path}')
    print('提示: 生成结果建议在 GIMP 里人工修一遍（补上激光扫不到的围栏、'
          '删掉观众席噪点），再用 RViz 叠 PCD 核对对齐。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
