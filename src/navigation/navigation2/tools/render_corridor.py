#!/usr/bin/env python3
"""
Render SFC corridor geometry into images (TDT-nav-kit-style colours).

绿 = 前端化简路径，蓝框 = SFC 走廊，红 = MINCO 优化轨迹。额外画：浅红 = 隧道
走廊项关闭，深红 = 打开；品红 = 隧道本体；青线 = 隧道轴线。

用法：
  python3 render_corridor.py <map.msgpack> <dump.json> <out_dir>
产出：
  sfc_corridor.png        全图：走廊框 + 前端路径 + 优化轨迹
  sfc_corridor_tunnel.png 隧道附近放大：关/开走廊项对比
  sfc_corridor_demo.png   受控实验：前端偏轴 0.2 m 时的关/开对比
"""
import json
import os
import sys

import msgpack
import numpy as np
from PIL import Image, ImageDraw

FLAT, OBSTACLE, TUNNEL, UNKNOWN = 0, 1, 2, 7

C_FREE = (255, 255, 255)
C_OBS = (0, 0, 0)
C_UNKNOWN = (205, 205, 205)
C_TUNNEL = (255, 214, 255)
C_BOX = (30, 90, 220)
C_FRONT = (20, 140, 40)
C_TRAJ_OFF = (240, 150, 150)
C_TRAJ_ON = (200, 20, 20)
C_START = (0, 160, 0)
C_GOAL = (0, 0, 220)
C_AXIS = (0, 170, 190)
C_WAYPOINT = (255, 140, 0)


def load_map(path):
    d = msgpack.unpackb(open(path, 'rb').read(), raw=False, strict_map_key=False)

    def chan(x):
        if isinstance(x, (bytes, bytearray)):
            return np.frombuffer(x, dtype=np.uint8)
        return np.array(x, dtype=np.uint8)

    w, h = d['width'], d['height']
    return {'w': w, 'h': h, 'res': d['resolution'],
            'ox': d['origin'][0], 'oy': d['origin'][1],
            'terrain': chan(d['terrain']).reshape(h, w),
            'ids': chan(d['tunnel_ids']).reshape(h, w)
            if 'tunnel_ids' in d else np.zeros((h, w), dtype=np.uint8)}


def w2p(m, x, y, scale):
    """Convert world coordinates to pixels (y axis flipped)."""
    return ((x - m['ox']) / m['res'] * scale,
            (m['h'] - (y - m['oy']) / m['res']) * scale)


def base_image(m, scale):
    t = m['terrain']
    rgb = np.empty((m['h'], m['w'], 3), dtype=np.uint8)
    rgb[...] = C_FREE
    rgb[t == OBSTACLE] = C_OBS
    rgb[t == UNKNOWN] = C_UNKNOWN
    rgb[t == TUNNEL] = C_TUNNEL
    img = Image.fromarray(rgb, 'RGB')
    if scale != 1:
        img = img.resize((m['w'] * scale, m['h'] * scale), Image.NEAREST)
    return img


def tunnel_axes(m):
    """Return each tunnel principal axis and half length."""
    axes = []
    for tid in (1, 2):
        ys, xs = np.where(m['ids'] == tid)
        if len(xs) < 3:
            continue
        pts = np.stack([m['ox'] + (xs + 0.5) * m['res'],
                        m['oy'] + (ys + 0.5) * m['res']], 1)
        c = pts.mean(0)
        u, s, vt = np.linalg.svd(pts - c)
        d = vt[0]
        hl = np.abs((pts - c) @ d).max()
        axes.append((c, d, hl))
    return axes


def draw_geometry(img, m, data, scale, traj_key, boxes=True, waypoints=False):
    d = ImageDraw.Draw(img, 'RGBA')

    if boxes:
        for (x0, y0, x1, y1) in data['boxes']:
            if x1 <= x0 or y1 <= y0:
                continue
            d.rectangle([w2p(m, x0, y1, scale), w2p(m, x1, y0, scale)],
                        outline=C_BOX, width=max(1, scale // 2))

    if waypoints and 'demo_waypoints' in data:
        for x, y in data['demo_waypoints']:
            px, py = w2p(m, x, y, scale)
            r = max(2, scale // 2)
            d.ellipse([px - r, py - r, px + r, py + r], fill=C_WAYPOINT)

    pts = [w2p(m, x, y, scale) for x, y in data['front_path']]
    if len(pts) > 1:
        d.line(pts, fill=C_FRONT, width=max(1, scale // 2))

    key = traj_key
    pts = [w2p(m, x, y, scale) for x, y in data[key]]
    if len(pts) > 1:
        color = C_TRAJ_ON if key.endswith('_on') else C_TRAJ_OFF
        d.line(pts, fill=color, width=max(2, scale))

    for k, color in (('start', C_START), ('goal', C_GOAL)):
        if k not in data:
            continue
        x, y = data[k]
        px, py = w2p(m, x, y, scale)
        r = 3 * scale
        d.ellipse([px - r, py - r, px + r, py + r], fill=color)
    return img


def draw_axes(img, m, scale, axes):
    d = ImageDraw.Draw(img)
    for c, dirv, hl in axes:
        a = c - dirv * (hl + 0.35)
        b = c + dirv * (hl + 0.35)
        d.line([w2p(m, a[0], a[1], scale), w2p(m, b[0], b[1], scale)],
               fill=C_AXIS, width=max(1, scale // 2))
    return img


def crop(img, m, cx, cy, span, scale):
    x0, y0 = w2p(m, cx - span, cy + span, scale)
    x1, y1 = w2p(m, cx + span, cy - span, scale)
    return img.crop((int(max(0, x0)), int(max(0, y0)),
                     int(min(img.width, x1)), int(min(img.height, y1))))


def panel_pair(m, data, scale, key_off, key_on, cx, cy, span, labels, boxes, wp=False):
    imgs = []
    for key in (key_off, key_on):
        img = base_image(m, scale)
        img = draw_geometry(img, m, data, scale, key, boxes=boxes, waypoints=wp)
        img = draw_axes(img, m, scale, tunnel_axes(m))
        imgs.append(crop(img, m, cx, cy, span, scale))
    off, on = imgs
    w = off.width + on.width + 12
    h = max(off.height, on.height) + 24
    canvas = Image.new('RGB', (w, h), (245, 245, 245))
    canvas.paste(off, (0, 24))
    canvas.paste(on, (off.width + 12, 24))
    d = ImageDraw.Draw(canvas)
    d.text((6, 6), labels[0], fill=(180, 60, 60))
    d.text((off.width + 18, 6), labels[1], fill=(160, 0, 0))
    return canvas


def main():
    map_path, dump_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out_dir, exist_ok=True)
    m = load_map(map_path)
    data = json.load(open(dump_path))
    scale = 4

    # 全图（TDT 示例图的画法）：底图 + 蓝框 + 绿路径 + 红轨迹
    img = base_image(m, scale)
    img = draw_geometry(img, m, data, scale, 'traj_corridor_on')
    img = draw_axes(img, m, scale, tunnel_axes(m))
    img.save(os.path.join(out_dir, 'sfc_corridor.png'))
    print('wrote sfc_corridor.png', img.size)

    axes = tunnel_axes(m)
    if axes:
        c, dirv, hl = axes[0]
        cx, cy, span = c[0], c[1], 1.9
    else:
        cx, cy, span = data['start'][0], data['start'][1], 2.0

    # 真实 A* 路径：隧道附近关/开
    panel_pair(m, data, scale, 'traj_corridor_off', 'traj_corridor_on',
               cx, cy, span, ('corridor OFF (real A* path)',
                              'corridor ON (real A* path)'), boxes=True
               ).save(os.path.join(out_dir, 'sfc_corridor_tunnel.png'))
    print('wrote sfc_corridor_tunnel.png')

    # 受控实验：前端偏轴
    if 'demo_traj_off' in data:
        panel_pair(m, data, scale, 'demo_traj_off', 'demo_traj_on',
                   cx, cy, span,
                   (f"corridor OFF (ref shifted {data.get('demo_lateral', 0):.2f} m)",
                    f"corridor ON (ref shifted {data.get('demo_lateral', 0):.2f} m)"),
                   boxes=True, wp=True
                   ).save(os.path.join(out_dir, 'sfc_corridor_demo.png'))
        print('wrote sfc_corridor_demo.png')


if __name__ == '__main__':
    main()
