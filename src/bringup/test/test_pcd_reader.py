"""pcd_reader 的单元测试。

只验证 read_pcd_xyz 的 ascii / binary 两条读取路径给出一致结果。PCD → msgpack
的转换链已随 pcd_to_navmap.py 删除，主链换成了 pgm_to_navmap.py。
"""

import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import pcd_reader  # noqa: E402


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


def test_ascii_and_binary_pcd_agree(tmp_path):
    xyz = make_scene()
    ascii_path = tmp_path / 'a.pcd'
    binary_path = tmp_path / 'b.pcd'
    write_ascii_pcd(ascii_path, xyz)
    write_binary_pcd(binary_path, xyz)

    from_ascii = pcd_reader.read_pcd_xyz(str(ascii_path))
    from_binary = pcd_reader.read_pcd_xyz(str(binary_path))

    assert from_ascii.shape == from_binary.shape == xyz.shape
    np.testing.assert_allclose(from_ascii, from_binary, atol=1e-6)
    np.testing.assert_allclose(from_binary, xyz, atol=1e-6)
