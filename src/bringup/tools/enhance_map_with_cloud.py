#!/usr/bin/env python3
"""用 3D 障碍点云给 slam_toolbox 的 PGM 补低矮障碍（20cm 高台等）。

背景：建图主链是 `/segmentation/obstacle -> pointcloud_to_laserscan -> /scan
-> slam_toolbox`。LaserScan 每 0.25° 只能保留一个最近距离，20cm 台面的顶面点
会被前沿/地面射线压掉，PGM 上只剩几个格子。这个脚本在建图时旁路订阅
`/segmentation/obstacle`，把**所有**落在“地面以上 min_height..max_height”的
点变换到 map 系、投影到与 `/map` 同分辨率的栅格，再叠加到 `/map` 上，
输出增强后的 PGM+YAML；之后照常 `pgm_to_navmap.py` 转 msgpack。

用法（mapping 正在跑时另开终端）：
  ros2 run bringup enhance_map_with_cloud.py \
      --sensor-height 0.175 --duration 20 \
      --output /tmp/RMUL_enhanced

参数里的高度都是“地面以上”：
  --min-height 0.03  地面残留以下不投
  --max-height 0.60  只补低矮障碍；高墙 slam_toolbox 本来就有
  --dilate 1         1 格膨胀，把稀疏台面边缘连成线
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np
import yaml

PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205
PIXEL_FREE = 254


def project_points_to_counts(
    points: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    ground_z: float,
    min_height: float,
    max_height: float,
    max_range: float = 0.0,
) -> np.ndarray:
    """把 map 系点 (N,3) 投影成每格点计数（行序：y 从 origin 向上）。

    用计数而不是 bool：单帧噪点/动态点只出现 1 次，静态台面会反复命中；
    调用方用 min_points 过滤，避免把移动物体糊成大片障碍。
    """
    counts = np.zeros((height, width), dtype=np.uint32)
    if points.size == 0:
        return counts
    z = points[:, 2] - ground_z
    keep = (z >= min_height) & (z <= max_height)
    selected = points[keep]
    if selected.size == 0:
        return counts
    if max_range > 0.0:
        # 只按 map 原点距离限制（建图区域通常就在原点附近），避免远处稀疏点。
        near = np.hypot(selected[:, 0], selected[:, 1]) <= max_range
        selected = selected[near]
    ix = np.floor((selected[:, 0] - origin_x) / resolution).astype(np.int64)
    iy = np.floor((selected[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    np.add.at(counts, (iy[valid], ix[valid]), 1)
    return counts


def project_points_to_mask(
    points: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    ground_z: float,
    min_height: float,
    max_height: float,
    min_points: int = 1,
    max_range: float = 0.0,
) -> np.ndarray:
    """兼容旧接口：计数 >= min_points 的格子。"""
    counts = project_points_to_counts(
        points, origin_x, origin_y, resolution, width, height, ground_z,
        min_height, max_height, max_range)
    return counts >= max(1, min_points)


def dilate_mask(mask: np.ndarray, cells: int) -> np.ndarray:
    """4 邻域膨胀 cells 次；不引入 scipy。"""
    result = mask.copy()
    for _ in range(max(0, cells)):
        padded = np.pad(result, 1, mode='constant', constant_values=False)
        result = (
            result
            | padded[:-2, 1:-1]
            | padded[2:, 1:-1]
            | padded[1:-1, :-2]
            | padded[1:-1, 2:]
        )
    return result


def merge_grid_to_image(
    data: np.ndarray,
    mask: np.ndarray,
    occupied: int = 100,
    unknown: int = -1,
) -> np.ndarray:
    """把 OccupancyGrid data 转成 PGM 灰度，再叠加 mask。

    data 行序：y 从 origin 向上；返回 PGM 行序（第 0 行是 y 最大），
    与 pgm_to_navmap.py 的读法一致。
    """
    image = np.full(data.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    image[data == 0] = PIXEL_FREE
    image[data == occupied] = PIXEL_OCCUPIED
    image[data == unknown] = PIXEL_UNKNOWN
    image[mask] = PIXEL_OCCUPIED
    return np.flipud(image)


def write_pgm(path: Path, image: np.ndarray) -> None:
    height, width = image.shape
    with open(path, 'wb') as stream:
        stream.write(b'P5\n')
        stream.write(b'# CREATOR: enhance_map_with_cloud.py\n')
        stream.write(f'{width} {height}\n255\n'.encode('ascii'))
        stream.write(image.tobytes())


# ---------------- ROS 侧（只在主程序里 import，便于单测） ----------------

def run(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node('enhance_map_with_cloud')
    map_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    state = {'map': None, 'counts': None, 'clouds': 0, 'points': 0}

    def on_map(msg: OccupancyGrid) -> None:
        state['map'] = msg

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)  # noqa: F841

    def on_cloud(msg: PointCloud2) -> None:
        current = state['map']
        if current is None or not msg.data:
            return
        try:
            tf = tf_buffer.lookup_transform(
                current.header.frame_id, msg.header.frame_id, msg.header.stamp,
                Duration(seconds=args.tf_timeout))
        except Exception as error:  # tf2 异常类型跨版本不稳定
            node.get_logger().warn(f'TF failed: {error}', throttle_duration_sec=2.0)
            return
        tr = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        translation = np.array([tr.x, tr.y, tr.z])
        count = msg.width
        points = np.empty((count, 3), dtype=np.float64)
        for i in range(count):
            px, py, pz = struct.unpack_from('<fff', msg.data, i * msg.point_step)
            points[i] = (px, py, pz)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite] @ rot.T + translation
        state['clouds'] += 1
        state['points'] += int(points.shape[0])

        info = current.info
        # 地面 z：当前机器人位姿 z - 雷达安装高度（平地建图；坡道需按段修正）。
        try:
            robot = tf_buffer.lookup_transform(
                current.header.frame_id, args.robot_frame, msg.header.stamp,
                Duration(seconds=args.tf_timeout))
            ground_z = (
                args.ground_z if args.ground_z is not None
                else robot.transform.translation.z - args.sensor_height)
        except Exception:
            return
        new_counts = project_points_to_counts(
            points, info.origin.position.x, info.origin.position.y,
            info.resolution, info.width, info.height, ground_z,
            args.min_height, args.max_height, args.max_range)
        if state['counts'] is None:
            state['counts'] = new_counts
        else:
            state['counts'] += new_counts

    node.create_subscription(OccupancyGrid, args.map_topic, on_map, map_qos)
    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, cloud_qos)

    deadline = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    current = state['map']
    if current is None:
        node.get_logger().error(f'没收到 {args.map_topic}，建图是否在跑？')
        rclpy.shutdown()
        return 1
    if state['counts'] is None:
        node.get_logger().error(f'没收到/没算出 {args.cloud_topic} 的有效投影')
        rclpy.shutdown()
        return 1

    info = current.info
    raw_mask = state['counts'] >= max(1, args.min_points)
    if not raw_mask.any():
        node.get_logger().error(
            f'投影后没有格子达到 min_points={args.min_points}，'
            '检查 --sensor-height / --min-height / --max-height')
        rclpy.shutdown()
        return 1
    mask = dilate_mask(raw_mask, args.dilate)
    data = np.asarray(current.data, dtype=np.int16).reshape(info.height, info.width)
    image = merge_grid_to_image(data, mask)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output.with_suffix('.pgm')
    yaml_path = output.with_suffix('.yaml')
    write_pgm(pgm_path, image)
    yaml_path.write_text(
        yaml.safe_dump({
            'image': pgm_path.name,
            'mode': 'trinary',
            'resolution': float(info.resolution),
            'origin': [float(info.origin.position.x), float(info.origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, sort_keys=False), encoding='utf-8')

    node.get_logger().info(
        f'云 {state["clouds"]} 帧/{state["points"]} 点，投影格子 {int(raw_mask.sum())}，'
        f'膨胀后 {int(mask.sum())}；写出 {pgm_path} + {yaml_path}')
    node.get_logger().info(
        f'下一步: python3 src/bringup/tools/pgm_to_navmap.py {yaml_path} -o <map>.msgpack')
    rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='用 3D 障碍云增强 slam_toolbox PGM')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--cloud-topic', default='/segmentation/obstacle')
    parser.add_argument('--robot-frame', default='base_link')
    parser.add_argument('--sensor-height', type=float, required=True,
                        help='雷达离地高度（仿真 0.175，实车按卷尺量）')
    parser.add_argument('--ground-z', type=float, default=None,
                        help='直接指定 map 系地面 z；坡道/地图 z 原点异常时用，'
                             '默认按 机器人z - sensor_height 逐帧算')
    parser.add_argument('--min-height', type=float, default=0.03,
                        help='地面以上最小高度（默认 0.03，滤掉地面残留）')
    parser.add_argument('--max-height', type=float, default=0.60,
                        help='地面以上最大高度（默认 0.60，只补低矮障碍）')
    parser.add_argument('--dilate', type=int, default=1, help='膨胀格数')
    parser.add_argument('--min-points', type=int, default=2,
                        help='同一格至少命中几次才保留（过滤动态/噪点）')
    parser.add_argument('--max-range', type=float, default=8.0,
                        help='只投影距 map 原点此半径内的点；0 关闭')
    parser.add_argument('--duration', type=float, default=15.0, help='采样秒数')
    parser.add_argument('--tf-timeout', type=float, default=0.2)
    parser.add_argument('--output', default='/tmp/map_enhanced',
                        help='输出前缀，生成 .pgm/.yaml')
    args = parser.parse_args()
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
