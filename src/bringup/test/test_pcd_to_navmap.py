"""pcd_to_navmap 的单元测试。

pcd_to_navmap 一步把 PCD 点云转成 rm_map_server 读的语义地图 msgpack，取代了
原先的 pcd_to_gridmap（出 pgm+yaml）+ pgm_to_navmap 两步链。重点验证几件容易
错、又不容易在 RViz 里看出来的事：
  1. PCD 的 ascii/binary 两条读取路径给出一致结果；
  2. 占据栅格 -> terrain 的行序不翻转（grid[0] 是 y 最小的一行，正好就是
     msgpack 的 index = y*width + x 布局，pgm 那次上下翻转不该再出现）；
  3. UNKNOWN 独立保留、不塌成 FLAT，否则规划器会以正常代价穿过没扫到的区域；
  4. 写出的 msgpack 能被 C++ 侧的 loadSemanticMap 接受（键齐全、通道等长）。
"""

import os
import struct
import sys

import msgpack
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import pcd_to_navmap as p2n  # noqa: E402


def write_ascii_pcd(path, xyz):
    with open(path, 'w') as out:
        out.write('# .PCD v0.7 - Point Cloud Data file format\n'
                  'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                  'COUNT 1 1 1\n')
        out.write(f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
        out.write(f'POINTS {len(xyz)}\nDATA ascii\n')
        for x, y, z in xyz:
            out.write(f'{x} {y} {z}\n')


def write_binary_pcd(path, xyz):
    with open(path, 'wb') as out:
        out.write(b'# .PCD v0.7 - Point Cloud Data file format\n'
                  b'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                  b'TYPE F F F F\nCOUNT 1 1 1 1\n')
        out.write(f'WIDTH {len(xyz)}\nHEIGHT 1\n'.encode())
        out.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        out.write(f'POINTS {len(xyz)}\nDATA binary\n'.encode())
        for x, y, z in xyz:
            out.write(struct.pack('<ffff', x, y, z, 1.0))


def make_scene():
    """一块 2m x 2m 的地面，加上一根落在 (1.5, 0.5) 的立柱。"""
    gx, gy = np.meshgrid(np.arange(0.0, 2.0, 0.05), np.arange(0.0, 2.0, 0.05))
    ground = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    pillar_z = np.arange(0.2, 1.0, 0.05)
    pillar = np.stack(
        [np.full(pillar_z.size, 1.5), np.full(pillar_z.size, 0.5), pillar_z], axis=1)
    return np.vstack([ground, pillar])


class Args:
    """凑齐 convert() 需要的字段，等价于 argparse 解析出的命名空间。"""

    def __init__(self, **kw):
        self.resolution = 0.05
        self.z_min = 0.15
        self.z_max = 1.20
        self.ground_z_max = 0.10
        self.min_points = 3
        self.padding = 0.5
        self.__dict__.update(kw)


def test_ascii_and_binary_pcd_agree(tmp_path):
    xyz = make_scene()
    ascii_path = tmp_path / 'a.pcd'
    binary_path = tmp_path / 'b.pcd'
    write_ascii_pcd(ascii_path, xyz)
    write_binary_pcd(binary_path, xyz)

    from_ascii = p2n.read_pcd_xyz(str(ascii_path))
    from_binary = p2n.read_pcd_xyz(str(binary_path))

    assert from_ascii.shape == from_binary.shape == xyz.shape
    np.testing.assert_allclose(from_ascii, from_binary, atol=1e-6)
    np.testing.assert_allclose(from_binary, xyz, atol=1e-6)


def test_pillar_becomes_occupied_and_ground_free():
    grid, origin_x, origin_y = p2n.build_grid(
        make_scene(), resolution=0.05, z_min=0.15, z_max=1.2,
        ground_z_max=0.10, min_points=3, padding=0.5)

    # 立柱所在格子应为占据。
    col = int((1.5 - origin_x) / 0.05)
    row = int((0.5 - origin_y) / 0.05)
    assert grid[row, col] == p2n.PIXEL_OCCUPIED

    # 地面中心应为空闲，地图外围留白应为未知。
    assert grid[int((1.0 - origin_y) / 0.05), int((1.0 - origin_x) / 0.05)] == \
        p2n.PIXEL_FREE
    assert grid[0, 0] == p2n.PIXEL_UNKNOWN


def test_grid_to_terrain_preserves_row_order_and_three_states():
    """terrain 不做 y 翻转，且三态一一对应。

    grid[0] 是 y 最小的一行，msgpack 的 index=y*width+x 也从 y=0 排起，所以
    grid_to_terrain 必须逐格照搬、不翻转。占据/空闲/未知三档各自映射到
    OBSTACLE/FLAT/UNKNOWN，UNKNOWN 尤其不能塌成 FLAT。
    """
    grid, _, _ = p2n.build_grid(
        make_scene(), resolution=0.05, z_min=0.15, z_max=1.2,
        ground_z_max=0.10, min_points=3, padding=0.5)
    terrain = p2n.grid_to_terrain(grid)

    assert terrain.shape == grid.shape
    np.testing.assert_array_equal(
        terrain == p2n.TERRAIN_OBSTACLE, grid == p2n.PIXEL_OCCUPIED)
    np.testing.assert_array_equal(
        terrain == p2n.TERRAIN_FLAT, grid == p2n.PIXEL_FREE)
    np.testing.assert_array_equal(
        terrain == p2n.TERRAIN_UNKNOWN, grid == p2n.PIXEL_UNKNOWN)
    # 未知格确实存在（外围留白），且没有被并进 FLAT。
    assert (terrain == p2n.TERRAIN_UNKNOWN).any()


def test_convert_builds_loadable_payload():
    """convert 产出的 payload 满足 C++ loadSemanticMap 的基本契约：
    键齐全、terrain/direction 等长且长度 = width*height、origin 三元、
    tunnels 为空、direction 全零（没有隧道格就不该带方向）。"""
    payload, info = p2n.convert(make_scene(), Args())

    for key in ('width', 'height', 'resolution', 'origin', 'terrain',
                'direction', 'tunnels'):
        assert key in payload, f'payload 缺少 {key}'

    cells = payload['width'] * payload['height']
    assert len(payload['terrain']) == cells
    assert len(payload['direction']) == cells
    assert payload['direction'] == bytes(cells)   # 全零
    assert payload['tunnels'] == []
    assert len(payload['origin']) == 3 and payload['origin'][2] == 0.0
    assert info['unknown'] > 0                     # UNKNOWN 保留


def test_convert_terrain_matches_grid_to_terrain():
    """convert 写进 payload 的 terrain 字节应与 grid_to_terrain 完全一致，
    确认中间没有多余的翻转或重排。"""
    args = Args()
    xyz = make_scene()
    grid, _, _ = p2n.build_grid(
        xyz, args.resolution, args.z_min, args.z_max, args.ground_z_max,
        args.min_points, args.padding)
    expected = p2n.grid_to_terrain(grid).tobytes()

    payload, _ = p2n.convert(xyz, args)
    assert payload['terrain'] == expected


def test_empty_cloud_raises():
    with pytest.raises(ValueError):
        p2n.build_grid(np.zeros((0, 3)), 0.05, 0.15, 1.2, 0.1, 3, 0.5)


def test_all_nan_cloud_raises():
    with pytest.raises(ValueError):
        p2n.build_grid(np.full((10, 3), np.nan), 0.05, 0.15, 1.2, 0.1, 3, 0.5)


def test_rejects_bad_arguments(tmp_path):
    path = tmp_path / 'a.pcd'
    write_ascii_pcd(path, make_scene())

    assert p2n.main([str(path), '-o', str(tmp_path / 'o.msgpack'),
                     '--z-min', '2.0', '--z-max', '1.0']) == 2
    assert p2n.main([str(path), '-o', str(tmp_path / 'o.msgpack'),
                     '--resolution', '0']) == 2


def test_main_writes_msgpack(tmp_path):
    path = tmp_path / 'a.pcd'
    write_ascii_pcd(path, make_scene())
    out = tmp_path / 'out.msgpack'
    assert p2n.main([str(path), '-o', str(out)]) == 0
    assert out.exists()

    payload = msgpack.unpackb(out.read_bytes(), raw=False)
    cells = payload['width'] * payload['height']
    assert len(payload['terrain']) == cells
    assert 'tunnel_ids' not in payload      # 没有隧道格就不写这个通道


def test_main_refuses_overwrite_without_force(tmp_path):
    path = tmp_path / 'a.pcd'
    write_ascii_pcd(path, make_scene())
    out = tmp_path / 'out.msgpack'
    assert p2n.main([str(path), '-o', str(out)]) == 0
    # 已存在且没给 -f，应拒绝并返回 1，不覆盖。
    assert p2n.main([str(path), '-o', str(out)]) == 1
    # 给了 -f 才覆盖。
    assert p2n.main([str(path), '-o', str(out), '-f']) == 0
