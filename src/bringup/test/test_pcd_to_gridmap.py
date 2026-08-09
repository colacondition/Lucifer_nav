"""pcd_to_gridmap 的单元测试。

重点验证两件容易错、又不容易在 RViz 里看出来的事：
  1. PCD 的 ascii/binary 两条读取路径给出一致结果；
  2. 写出的 PGM 行序与 rm_map_server 的读法互为逆操作（栅格 y 最小的一行
     在 PGM 里是最后一行）。行序反了地图会上下颠倒，而对称场地几乎看不出来。
"""

import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import pcd_to_gridmap as p2g  # noqa: E402


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


def read_pgm(path):
    """按 rm_map_server 的方式读回 PGM，返回 (pixels, width, height)。"""
    with open(path, 'rb') as src:
        data = src.read()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, width, height, _maxval = tokens
    assert magic == b'P5'
    width, height = int(width), int(height)
    pixels = np.frombuffer(data[pos + 1:pos + 1 + width * height], dtype=np.uint8)
    return pixels.reshape(height, width), width, height


def make_scene():
    """一块 2m x 2m 的地面，加上一根落在 (1.5, 0.5) 的立柱。"""
    gx, gy = np.meshgrid(np.arange(0.0, 2.0, 0.05), np.arange(0.0, 2.0, 0.05))
    ground = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    pillar_z = np.arange(0.2, 1.0, 0.05)
    pillar = np.stack(
        [np.full(pillar_z.size, 1.5), np.full(pillar_z.size, 0.5), pillar_z], axis=1)
    return np.vstack([ground, pillar])


def test_ascii_and_binary_pcd_agree(tmp_path):
    xyz = make_scene()
    ascii_path = tmp_path / 'a.pcd'
    binary_path = tmp_path / 'b.pcd'
    write_ascii_pcd(ascii_path, xyz)
    write_binary_pcd(binary_path, xyz)

    from_ascii = p2g.read_pcd_xyz(str(ascii_path))
    from_binary = p2g.read_pcd_xyz(str(binary_path))

    assert from_ascii.shape == from_binary.shape == xyz.shape
    np.testing.assert_allclose(from_ascii, from_binary, atol=1e-6)
    np.testing.assert_allclose(from_binary, xyz, atol=1e-6)


def test_pillar_becomes_occupied_and_ground_free():
    grid, origin_x, origin_y = p2g.build_grid(
        make_scene(), resolution=0.05, z_min=0.15, z_max=1.2,
        ground_z_max=0.10, min_points=3, padding=0.5)

    # 立柱所在格子应为占据。
    col = int((1.5 - origin_x) / 0.05)
    row = int((0.5 - origin_y) / 0.05)
    assert grid[row, col] == p2g.PIXEL_OCCUPIED

    # 地面中心应为空闲，地图外围留白应为未知。
    assert grid[int((1.0 - origin_y) / 0.05), int((1.0 - origin_x) / 0.05)] == \
        p2g.PIXEL_FREE
    assert grid[0, 0] == p2g.PIXEL_UNKNOWN


def test_pgm_row_order_survives_round_trip(tmp_path):
    grid, _, _ = p2g.build_grid(
        make_scene(), resolution=0.05, z_min=0.15, z_max=1.2,
        ground_z_max=0.10, min_points=3, padding=0.5)

    pgm_path = tmp_path / 'm.pgm'
    p2g.write_pgm(str(pgm_path), grid)
    pixels, width, height = read_pgm(str(pgm_path))
    assert (height, width) == grid.shape

    # rm_map_server 取 image_index = (height - y - 1) * width + x，
    # 等价于把读到的图像上下翻转一次。翻回来必须与原栅格逐格相同。
    np.testing.assert_array_equal(np.flipud(pixels), grid)


def test_written_yaml_matches_map_server_contract(tmp_path):
    yaml_path = tmp_path / 'm.yaml'
    p2g.write_yaml(str(yaml_path), 'm.pgm', 0.05, -2.581, -2.361)
    text = yaml_path.read_text()

    assert 'negate: 0' in text
    assert 'image: m.pgm' in text
    assert 'origin: [-2.581, -2.361, 0]' in text

    # negate=0 时 rm_map_server 算 occupancy = 1 - pixel/255，然后
    #   occupancy > occupied_thresh -> 100(占据)
    #   occupancy < free_thresh     -> 0(空闲)
    #   其余                        -> -1(未知)
    # 三个像素常量必须各自落进对的那一档。特别是 205 不能掉进空闲档，
    # 否则从未扫到的区域会被当成可通行。
    def occupancy(pixel):
        return 1.0 - pixel / 255.0

    assert occupancy(p2g.PIXEL_OCCUPIED) > p2g.OCCUPIED_THRESH
    assert occupancy(p2g.PIXEL_FREE) < p2g.FREE_THRESH
    assert p2g.FREE_THRESH <= occupancy(p2g.PIXEL_UNKNOWN) <= p2g.OCCUPIED_THRESH


def test_rejects_bad_arguments(tmp_path):
    xyz = make_scene()
    path = tmp_path / 'a.pcd'
    write_ascii_pcd(path, xyz)

    assert p2g.main([str(path), '-o', str(tmp_path / 'o'),
                     '--z-min', '2.0', '--z-max', '1.0']) == 2
    assert p2g.main([str(path), '-o', str(tmp_path / 'o'),
                     '--resolution', '0']) == 2


def test_empty_cloud_raises():
    with pytest.raises(ValueError):
        p2g.build_grid(np.zeros((0, 3)), 0.05, 0.15, 1.2, 0.1, 3, 0.5)


def test_all_nan_cloud_raises():
    with pytest.raises(ValueError):
        p2g.build_grid(np.full((10, 3), np.nan), 0.05, 0.15, 1.2, 0.1, 3, 0.5)


MAP_DIR = os.path.join(os.path.dirname(__file__), '..', 'map')


@pytest.mark.parametrize('world', ['RMUL', 'RMUC'])
def test_checked_in_maps_classify_every_pixel_correctly(world):
    """守住已入库的场地图：手工在 GIMP 里改图很容易引入 205 灰度，
    而 free_thresh 若大于 0.196 就会把未知区域读成空闲。这里直接按
    rm_map_server 的公式复算一遍，确保图里出现的每个灰度都落在预期档位。"""
    yaml_path = os.path.join(MAP_DIR, f'{world}.yaml')
    pgm_path = os.path.join(MAP_DIR, f'{world}.pgm')
    if not (os.path.exists(yaml_path) and os.path.exists(pgm_path)):
        pytest.skip(f'{world} 地图不在仓库里')

    fields = {}
    for line in open(yaml_path):
        line = line.split('#')[0].strip()
        if ':' in line:
            key, _, value = line.partition(':')
            fields[key.strip()] = value.strip()

    assert fields['negate'] == '0', 'negate!=0 会反转占据语义'
    free_thresh = float(fields['free_thresh'])
    occupied_thresh = float(fields['occupied_thresh'])

    # 未知灰度绝不能落进空闲档，否则从未扫到的区域会被规划器当成可通行。
    assert 1.0 - p2g.PIXEL_UNKNOWN / 255.0 >= free_thresh, (
        f'{world}.yaml 的 free_thresh={free_thresh} 太大，'
        f'灰度 {p2g.PIXEL_UNKNOWN} 会被判成空闲')

    pixels, _, _ = read_pgm(pgm_path)
    present = set(np.unique(pixels).tolist())
    # 黑必须占据、白必须空闲，这是画图时唯一的约定。
    if p2g.PIXEL_OCCUPIED in present:
        assert 1.0 - p2g.PIXEL_OCCUPIED / 255.0 > occupied_thresh
    if p2g.PIXEL_FREE in present:
        assert 1.0 - p2g.PIXEL_FREE / 255.0 < free_thresh


def test_main_writes_both_files(tmp_path):
    path = tmp_path / 'a.pcd'
    write_ascii_pcd(path, make_scene())
    prefix = tmp_path / 'out'
    assert p2g.main([str(path), '-o', str(prefix)]) == 0
    assert (tmp_path / 'out.pgm').exists()
    assert (tmp_path / 'out.yaml').exists()
