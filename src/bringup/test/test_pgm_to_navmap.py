"""pgm_to_navmap 的单元测试。

pgm_to_navmap 把 slam_toolbox 建图、map_saver_cli 存出的 pgm+yaml 转成
rm_map_server 读的语义地图 msgpack。重点验证几件容易错、又不容易在 RViz
里看出来的事：
  1. P5 二进制 / P2 ascii 两条读取路径给出一致结果；
  2. 行序翻转恰好做一次：pgm 第 0 行是图像顶部（y 最大），msgpack 的
     index = y*width + x 从 y 最小行排起；
  3. 未知灰度 205 的占据度 50/255≈0.19608 恰好卡在默认 free_thresh=0.196
     之上，阈值必须用严格不等号，否则未知塌成空闲；
  4. 写出的 msgpack 满足 C++ loadSemanticMap 的契约（键齐全、通道等长）。
"""

import os
import sys

import msgpack
import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import pgm_to_navmap as g2n  # noqa: E402

PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205
PIXEL_FREE = 254


def make_image():
    """3x4 小图：顶行全占据，中间空闲夹一个未知，底行全空闲。

    pgm 行序是图像自顶向下，所以第 0 行（占据）对应地图 y 最大的一行。
    """
    return np.array([
        [PIXEL_OCCUPIED, PIXEL_OCCUPIED, PIXEL_OCCUPIED, PIXEL_OCCUPIED],
        [PIXEL_FREE, PIXEL_UNKNOWN, PIXEL_FREE, PIXEL_FREE],
        [PIXEL_FREE, PIXEL_FREE, PIXEL_FREE, PIXEL_FREE],
    ], dtype=np.uint8)


def write_p5(path, img, maxval=255, comment=False):
    with open(path, 'wb') as out:
        out.write(b'P5\n')
        if comment:
            out.write(b'# CREATOR: map_saver_cli 0.05 m/pix\n')
        out.write(f'{img.shape[1]} {img.shape[0]}\n{maxval}\n'.encode())
        out.write(img.tobytes())


def write_p2(path, img, maxval=255):
    with open(path, 'w') as out:
        out.write(f'P2\n{img.shape[1]} {img.shape[0]}\n{maxval}\n')
        for row in img:
            out.write(' '.join(str(v) for v in row) + '\n')


def write_map_yaml(path, image_name, **overrides):
    meta = {
        'image': image_name,
        'mode': 'trinary',
        'resolution': 0.05,
        'origin': [-1.97, -1.86, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    meta.update(overrides)
    with open(path, 'w') as out:
        yaml.safe_dump(meta, out)


def test_p5_and_p2_agree(tmp_path):
    img = make_image()
    p5 = tmp_path / 'a.pgm'
    p2 = tmp_path / 'b.pgm'
    write_p5(p5, img, comment=True)   # map_saver_cli 会写 # 注释行
    write_p2(p2, img)

    from_p5, maxval5 = g2n.read_pgm(str(p5))
    from_p2, maxval2 = g2n.read_pgm(str(p2))

    assert maxval5 == maxval2 == 255
    np.testing.assert_array_equal(from_p5, from_p2)
    np.testing.assert_array_equal(from_p5, img)


def test_terrain_flips_row_order():
    """pgm 顶行（占据）必须翻到 terrain 的最后一行（y 最大）。"""
    terrain = g2n.pgm_to_terrain(make_image(), 255, negate=0,
                                 occupied_thresh=0.65, free_thresh=0.196)

    assert (terrain[-1] == g2n.TERRAIN_OBSTACLE).all()   # 图像顶部 -> y 最大
    assert (terrain[0] == g2n.TERRAIN_FLAT).all()        # 图像底部 -> y 最小
    # 未知格跟着中间行走：原图第 1 行，翻转后是倒数第 2 行。
    assert terrain[-2, 1] == g2n.TERRAIN_UNKNOWN


def test_unknown_205_stays_unknown_with_default_thresholds():
    """205 的占据度 0.19608 > free_thresh 0.196，必须留在 UNKNOWN。"""
    img = np.full((2, 2), PIXEL_UNKNOWN, dtype=np.uint8)
    terrain = g2n.pgm_to_terrain(img, 255, negate=0,
                                 occupied_thresh=0.65, free_thresh=0.196)
    assert (terrain == g2n.TERRAIN_UNKNOWN).all()


def test_negate_inverts_occupancy():
    img = make_image()
    normal = g2n.pgm_to_terrain(img, 255, 0, 0.65, 0.196)
    negated = g2n.pgm_to_terrain(255 - img, 255, 1, 0.65, 0.196)
    np.testing.assert_array_equal(normal, negated)


def test_convert_builds_loadable_payload(tmp_path):
    """payload 满足 C++ loadSemanticMap 的基本契约：键齐全、terrain/direction
    等长且长度 = width*height、origin 三元、tunnels 为空、direction 全零。"""
    pgm = tmp_path / 'm.pgm'
    map_yaml = tmp_path / 'm.yaml'
    write_p5(pgm, make_image())
    write_map_yaml(map_yaml, 'm.pgm')

    meta = g2n.load_map_yaml(str(map_yaml))
    img, maxval = g2n.read_pgm(meta['image'])
    payload, info = g2n.convert(meta, img, maxval)

    for key in ('width', 'height', 'resolution', 'origin', 'terrain',
                'direction', 'tunnels'):
        assert key in payload, f'payload 缺少 {key}'

    cells = payload['width'] * payload['height']
    assert (payload['width'], payload['height']) == (4, 3)
    assert len(payload['terrain']) == cells
    assert len(payload['direction']) == cells
    assert payload['direction'] == bytes(cells)   # 全零
    assert payload['tunnels'] == []
    assert payload['origin'] == [-1.97, -1.86, 0.0]
    assert info['occupied'] == 4 and info['unknown'] == 1 and info['free'] == 7


def test_yaml_relative_image_resolved_from_yaml_dir(tmp_path):
    """image 是相对路径时按 yaml 所在目录解析（map_saver_cli 的存法）。"""
    sub = tmp_path / 'maps'
    sub.mkdir()
    write_p5(sub / 'w.pgm', make_image())
    write_map_yaml(sub / 'w.yaml', 'w.pgm')

    meta = g2n.load_map_yaml(str(sub / 'w.yaml'))
    assert meta['image'] == sub / 'w.pgm'


def test_yaml_missing_keys_raise(tmp_path):
    bad = tmp_path / 'bad.yaml'
    with open(bad, 'w') as out:
        yaml.safe_dump({'image': 'x.pgm', 'resolution': 0.05}, out)  # 缺 origin
    with pytest.raises(ValueError):
        g2n.load_map_yaml(str(bad))


def test_rejects_non_pgm(tmp_path):
    fake = tmp_path / 'x.pgm'
    fake.write_bytes(b'P6\n1 1\n255\n\x00\x00\x00')
    with pytest.raises(ValueError):
        g2n.read_pgm(str(fake))


def test_main_writes_msgpack(tmp_path):
    pgm = tmp_path / 'm.pgm'
    map_yaml = tmp_path / 'm.yaml'
    write_p5(pgm, make_image())
    write_map_yaml(map_yaml, 'm.pgm')
    out = tmp_path / 'out.msgpack'

    assert g2n.main([str(map_yaml), '-o', str(out)]) == 0
    assert out.exists()

    payload = msgpack.unpackb(out.read_bytes(), raw=False)
    cells = payload['width'] * payload['height']
    assert len(payload['terrain']) == cells
    assert 'tunnel_ids' not in payload      # 没有隧道格就不写这个通道
    # 行序抽查：msgpack 第 0 行（y 最小）应是 pgm 的底行（空闲）。
    terrain = np.frombuffer(payload['terrain'], dtype=np.uint8).reshape(
        payload['height'], payload['width'])
    assert (terrain[0] == g2n.TERRAIN_FLAT).all()
    assert (terrain[-1] == g2n.TERRAIN_OBSTACLE).all()


def test_main_refuses_overwrite_without_force(tmp_path):
    pgm = tmp_path / 'm.pgm'
    map_yaml = tmp_path / 'm.yaml'
    write_p5(pgm, make_image())
    write_map_yaml(map_yaml, 'm.pgm')
    out = tmp_path / 'out.msgpack'

    assert g2n.main([str(map_yaml), '-o', str(out)]) == 0
    # 已存在且没给 -f，应拒绝并返回 1，不覆盖。
    assert g2n.main([str(map_yaml), '-o', str(out)]) == 1
    # 给了 -f 才覆盖。
    assert g2n.main([str(map_yaml), '-o', str(out), '-f']) == 0


def test_default_output_next_to_yaml(tmp_path):
    pgm = tmp_path / 'm.pgm'
    map_yaml = tmp_path / 'm.yaml'
    write_p5(pgm, make_image())
    write_map_yaml(map_yaml, 'm.pgm')

    assert g2n.main([str(map_yaml)]) == 0
    assert (tmp_path / 'm.msgpack').exists()
