#!/usr/bin/env python3
"""把 slam_toolbox 建图、map_saver_cli 存出的 pgm+yaml 转成 rm_map_server 读的语义地图 msgpack。

为什么有这条链：原来的 PCD 直转链（全局高度切片）对地面起伏大的点云会满图误判
（实测 RMUL.pcd 地面起伏 ~0.4 m，任何全局 z 阈值都同时切在某些区域的地面上和
另一些区域的墙下面）。slam_toolbox 逐帧做射线更新，激光穿过的格子被反复标空闲，
孤立噪点自然被洗掉，输出的栅格干净得多。完整流程：
    mode:=mapping 起 slam_toolbox（见 real/sim launch 的建图链一节）
    ros2 run nav2_map_server map_saver_cli -f src/bringup/map/<world>
    python3 pgm_to_navmap.py ../map/<world>.yaml
    semantic_map_editor.py 人工标隧道

行序是唯一的坑：pgm 第 0 行是图像顶部（y 最大的一行），msgpack 的 terrain 是
index = y*width + x、从 y 最小行排起，所以必须上下翻转一次（PCD 直转链没有
pgm 这一步，就不需要翻转）。

灰度→占据度沿用 nav2 map_server 的 trinary 约定（negate=0 时
occ = (maxval-pixel)/maxval）：
    occ > occupied_thresh → OBSTACLE(1)
    occ < free_thresh     → FLAT(0)
    其间（如灰度 205）    → UNKNOWN(7)
UNKNOWN 必须独立保留、不能塌成 FLAT：从未扫到的区域要吃 A* 的 unknown_cost
惩罚（见 semantic_map.hpp 的 TerrainType）。

隧道格这一步一律留空（direction 全零、tunnels 为空），必须人工在编辑器里标。

只依赖 numpy + msgpack + pyyaml。用法：
    python3 pgm_to_navmap.py RMUL.yaml -o ../map/RMUL.msgpack
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import msgpack
import numpy as np
import yaml

# 必须与 semantic_map.hpp 的 TerrainType 一致。
TERRAIN_FLAT = 0
TERRAIN_OBSTACLE = 1
TERRAIN_UNKNOWN = 7


def _next_token(stream) -> bytes:
    """读下一个 PGM 头 token，跳过空白与 # 注释行。"""
    token = b''
    while True:
        ch = stream.read(1)
        if not ch:
            if token:
                return token
            raise ValueError('PGM 头不完整')
        if ch == b'#' and not token:
            stream.readline()
            continue
        if ch.isspace():
            if token:
                return token
            continue
        token += ch


def read_pgm(path):
    """读 PGM，返回 (二维 uint8 数组, maxval)。支持 P5 二进制 / P2 ascii。

    map_saver_cli 存的是 P5；P2 只是为了方便手工构造测试用例。
    """
    with open(path, 'rb') as stream:
        magic = _next_token(stream)
        if magic not in (b'P5', b'P2'):
            raise ValueError(
                f'不是 PGM 文件（magic={magic!r}），map_saver_cli 存出的应是 P5')
        width = int(_next_token(stream))
        height = int(_next_token(stream))
        maxval = int(_next_token(stream))
        if width <= 0 or height <= 0:
            raise ValueError(f'PGM 尺寸非法: {width}x{height}')
        if not 0 < maxval <= 255:
            raise ValueError(f'不支持 maxval={maxval} 的 PGM（应为 1..255，单字节像素）')

        if magic == b'P5':
            raw = stream.read(width * height)
            if len(raw) != width * height:
                raise ValueError('PGM 数据不足 width*height 字节')
            img = np.frombuffer(raw, dtype=np.uint8)
        else:
            img = np.loadtxt(stream, dtype=np.uint8).ravel()
            if img.size != width * height:
                raise ValueError('PGM(P2) 数据个数与 width*height 不符')
        return img.reshape(height, width), maxval


def load_map_yaml(path):
    """读 map_saver_cli 输出的地图 yaml，返回标准化后的字段字典。"""
    path = Path(path)
    with open(path) as stream:
        meta = yaml.safe_load(stream)
    if not isinstance(meta, dict):
        raise ValueError('地图 yaml 不是键值映射')
    for key in ('image', 'resolution', 'origin'):
        if key not in meta:
            raise ValueError(f'地图 yaml 缺少 {key} 字段')

    image = Path(str(meta['image']))
    if not image.is_absolute():
        image = path.parent / image

    origin = list(meta['origin'])
    if len(origin) < 2:
        raise ValueError('origin 至少要有 [x, y]')

    resolution = float(meta['resolution'])
    if resolution <= 0.0:
        raise ValueError('resolution 必须为正')

    return {
        'image': image,
        'resolution': resolution,
        'origin_x': float(origin[0]),
        'origin_y': float(origin[1]),
        'yaw': float(origin[2]) if len(origin) >= 3 else 0.0,
        'negate': int(meta.get('negate', 0)),
        'occupied_thresh': float(meta.get('occupied_thresh', 0.65)),
        'free_thresh': float(meta.get('free_thresh', 0.196)),
    }


def pgm_to_terrain(img, maxval, negate, occupied_thresh, free_thresh):
    """灰度 → terrain 三态标签，并做 pgm→msgpack 的行序翻转。

    阈值判定与 nav2 map_server 的 trinary 模式一致：严格大于 occupied_thresh
    才占据、严格小于 free_thresh 才空闲。未知灰度 205 的占据度是 50/255≈0.19608，
    恰好卡在默认 free_thresh=0.196 之上 —— 用 <= 就会把未知塌成空闲。
    """
    scale = img.astype(np.float64) / float(maxval)
    occupancy = scale if negate else 1.0 - scale

    terrain = np.full(img.shape, TERRAIN_UNKNOWN, dtype=np.uint8)
    terrain[occupancy > occupied_thresh] = TERRAIN_OBSTACLE
    terrain[occupancy < free_thresh] = TERRAIN_FLAT
    # pgm 第 0 行是图像顶部（y 最大），msgpack 的 index=y*width+x 从 y 最小行排起。
    return np.flipud(terrain)


def convert(meta, img, maxval):
    terrain = pgm_to_terrain(
        img, maxval, meta['negate'], meta['occupied_thresh'], meta['free_thresh'])
    height, width = terrain.shape

    payload = {
        "width": width,
        "height": height,
        "resolution": meta['resolution'],
        "origin": [meta['origin_x'], meta['origin_y'], 0.0],
        # C-order tobytes(): index = y*width + x，与 map_server_node.cpp 一致。
        "terrain": np.ascontiguousarray(terrain).tobytes(),
        # 没有隧道格，方向通道全零。C++ 侧会校验「非方向标签不得带方向」。
        "direction": bytes(width * height),
        "tunnels": [],
    }
    info = {
        "width": width,
        "height": height,
        "origin": (meta['origin_x'], meta['origin_y']),
        "occupied": int((terrain == TERRAIN_OBSTACLE).sum()),
        "free": int((terrain == TERRAIN_FLAT).sum()),
        "unknown": int((terrain == TERRAIN_UNKNOWN).sum()),
    }
    return payload, info


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="把 map_saver_cli 存出的 pgm+yaml 转成 rm_map_server 用的语义地图 msgpack",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("yaml", help="map_saver_cli 输出的地图 yaml（内含 pgm 路径与阈值）")
    parser.add_argument(
        "-o", "--output", default=None,
        help="输出 msgpack 路径，默认与 yaml 同名同目录（换成 .msgpack 后缀）")
    parser.add_argument("-f", "--force", action="store_true",
                        help="覆盖已存在的输出文件")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    output = Path(args.output) if args.output else Path(args.yaml).with_suffix(".msgpack")
    if output.exists() and not args.force:
        print(f"输出已存在，用 -f 覆盖: {output}", file=sys.stderr)
        return 1

    try:
        meta = load_map_yaml(args.yaml)
        img, maxval = read_pgm(meta['image'])
        payload, info = convert(meta, img, maxval)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"转换失败: {error}", file=sys.stderr)
        return 1

    if abs(meta['yaw']) > 1e-9:
        print(f"警告: yaml 的 origin 带非零 yaw={meta['yaw']:.4f}，msgpack 地图没有"
              "旋转通道，将按 0 处理。建图时保持地图不旋转，或先重存一张 yaw=0 的图。",
              file=sys.stderr)

    output.write_bytes(msgpack.packb(payload, use_bin_type=True))

    total = info["width"] * info["height"]
    print(f"读入 {info['width']}x{info['height']} pgm -> msgpack "
          f"@ {meta['resolution']} m/格")
    print(f"  origin=({info['origin'][0]:.3f}, {info['origin'][1]:.3f})")
    print(f"  obstacle {info['occupied']:8d} ({100.0 * info['occupied'] / total:5.2f}%)")
    print(f"  flat     {info['free']:8d} ({100.0 * info['free'] / total:5.2f}%)")
    print(f"  unknown  {info['unknown']:8d} ({100.0 * info['unknown'] / total:5.2f}%)")
    print("  tunnel          0 (栅格区分不出，用 semantic_map_editor.py 人工标)")
    print(f"  写出 {output}")
    print("提示: 生成后建议用 semantic_map_editor.py 核对，补上激光扫不到的围栏，"
          "并标注隧道。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
