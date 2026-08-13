#!/usr/bin/env python3
"""纯 numpy 的 PCD xyz 读取器（无 open3d 依赖）。

从已删除的 pcd_to_navmap.py 拆出：PCD → msgpack 的转换链已废弃（主链换成
slam_toolbox 建图 + pgm_to_navmap.py），但 semantic_map_editor 载入 PCD 做
高度底图还需要读 xyz，所以把 read_pcd_xyz 独立成这个小模块。

支持 ascii / binary 两种 DATA 布局。binary_compressed（LZF）标准库没有解码器，
直接报错并给出转换命令，而不是悄悄返回错的点。
"""

from __future__ import annotations

import numpy as np

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
