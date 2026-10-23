"""clean_map_dynamic.py 的单元测试（合成数据，不依赖真实地图/ROS）。

场景：10m x 10m x 3m 房间（地面 + 四墙 + 天花板）作为静态结构；房间正中央
悬浮一个 0.6m x 0.6m x 1.2m 的方块（模拟走过的人/车留下的动态残影），它只在
12 个关键帧中的 1 帧观测里出现。最终地图 = 房间 + 动态方块 + 一撮远处无观测点。

关键帧点云按 T_world_imu 变换到 IMU 系落盘（与 small_glim
mapping.save_raw_mapping_frames:=true 的产物同构），poses.txt 用
se3(tx,ty,tz,qx,qy,qz,qw) 六位小数格式。

断言：
  * 动态方块点删除率 >= 80%；
  * 静态房间点保留率 >= 95%；
  * 远处无观测点 100% 保留（没有证据就不删）；
  * 输出/removed PCD 可被 pcd_reader 读回，且保留 + 删除 == 输入。
"""

import os
import sys

import numpy as np
import pytest

_TOOLS_DIR = os.path.join(os.path.dirname(__file__), '..', 'tools')
sys.path.insert(0, os.path.abspath(_TOOLS_DIR))

import clean_map_dynamic as cmd  # noqa: E402
import pcd_reader  # noqa: E402

ROOM_MIN = np.array([0.0, 0.0, 0.0])
ROOM_MAX = np.array([10.0, 10.0, 3.0])
DEFAULT_SPACING = 0.1
NUM_KEYFRAMES = 12
DYNAMIC_FRAME = 4          # 方块只在这一个关键帧里被观测到
BLOCK_CENTER_XY = (5.0, 5.0)
BLOCK_HALF_XY = 0.3
BLOCK_Z = (0.3, 1.5)
FAR_CLUSTER_OFFSET = np.array([60.0, 60.0, 0.0])


# ---------------------------------------------------------------------------
# 合成数据
# ---------------------------------------------------------------------------
def make_room_points(spacing=DEFAULT_SPACING):
    """房间的 6 个面（地面、天花板、四墙）按 grid 采样，返回世界系 (N,3)。"""
    xs = np.arange(ROOM_MIN[0], ROOM_MAX[0] + 1e-9, spacing)
    ys = np.arange(ROOM_MIN[1], ROOM_MAX[1] + 1e-9, spacing)
    zs = np.arange(ROOM_MIN[2], ROOM_MAX[2] + 1e-9, spacing)
    parts = []

    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    plane = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
    parts.append(plane)                                          # 地面 z=0
    parts.append(np.column_stack(
        [gx.ravel(), gy.ravel(), np.full(gx.size, ROOM_MAX[2])]))  # 天花板

    gyz, gzz = np.meshgrid(ys, zs, indexing='ij')
    parts.append(np.column_stack([np.full(gyz.size, ROOM_MIN[0]),
                                  gyz.ravel(), gzz.ravel()]))    # x=0 墙
    parts.append(np.column_stack([np.full(gyz.size, ROOM_MAX[0]),
                                  gyz.ravel(), gzz.ravel()]))    # x=10 墙

    gxz, gzx = np.meshgrid(xs, zs, indexing='ij')
    parts.append(np.column_stack([gxz.ravel(),
                                  np.full(gxz.size, ROOM_MIN[1]),
                                  gzx.ravel()]))                 # y=0 墙
    parts.append(np.column_stack([gxz.ravel(),
                                  np.full(gxz.size, ROOM_MAX[1]),
                                  gzx.ravel()]))                 # y=10 墙
    return np.vstack(parts)


def make_block_points(spacing=DEFAULT_SPACING):
    """方块 6 个面的采样点（世界系），模拟动态残影。"""
    cx, cy = BLOCK_CENTER_XY
    hx = hy = BLOCK_HALF_XY
    z0, z1 = BLOCK_Z
    xs = np.arange(-hx, hx + 1e-9, spacing)
    ys = np.arange(-hy, hy + 1e-9, spacing)
    zs = np.arange(z0, z1 + 1e-9, spacing)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    gxz, gzz = np.meshgrid(xs, zs, indexing='ij')
    gyz, gzz2 = np.meshgrid(ys, zs, indexing='ij')
    parts = [
        np.column_stack([cx + gx.ravel(), cy + gy.ravel(), np.full(gx.size, z1)]),
        np.column_stack([cx + gx.ravel(), cy + gy.ravel(), np.full(gx.size, z0)]),
        np.column_stack([cx + np.full(gxz.size, hx), cy + gxz.ravel(), gzz.ravel()]),
        np.column_stack([cx + np.full(gxz.size, -hx), cy + gxz.ravel(), gzz.ravel()]),
        np.column_stack([cx + gyz.ravel(), cy + np.full(gyz.size, hy), gzz2.ravel()]),
        np.column_stack([cx + gyz.ravel(), cy + np.full(gyz.size, -hy), gzz2.ravel()]),
    ]
    return np.vstack(parts)


def make_far_cluster(count=64):
    """远离所有关键帧量程的一撮点：任何关键帧都不会给它证据，应原样保留。"""
    offsets = np.arange(count, dtype=np.float64) * 0.05
    return np.column_stack([
        FAR_CLUSTER_OFFSET[0] + offsets,
        np.full(count, FAR_CLUSTER_OFFSET[1]),
        np.full(count, FAR_CLUSTER_OFFSET[2]),
    ])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def make_poses(num=NUM_KEYFRAMES, radius=3.2, height=1.0):
    """环绕房间中心的 12 个位姿：返回 [(t(3,), R(3,3), quat(4,)), ...]。"""
    poses = []
    for k in range(num):
        theta = 2.0 * np.pi * k / num
        t = np.array([
            BLOCK_CENTER_XY[0] + radius * np.cos(theta),
            BLOCK_CENTER_XY[1] + radius * np.sin(theta),
            height + 0.1 * np.sin(2.0 * theta),
        ])
        yaw = theta + np.pi + 0.1                  # x 轴大体朝向房间中心
        pitch = np.deg2rad(5.0 if k % 2 else -5.0)
        qz = np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])
        qy = np.array([0.0, np.sin(pitch / 2.0), 0.0, np.cos(pitch / 2.0)])
        quat = _quat_mul(qz, qy)
        rot = cmd._quat_to_rot(quat.reshape(1, 4))[0]
        poses.append((t, rot, quat))
    return poses


def write_scene(tmp_path, dynamic_frame=DYNAMIC_FRAME):
    """落盘 world.pcd + keyframes/{frame_*.pcd, poses.txt}，返回路径与真值。"""
    static = make_room_points()
    block = make_block_points()
    far = make_far_cluster()
    map_points = np.vstack([static, block, far])
    poses = make_poses()

    map_path = tmp_path / 'world.pcd'
    cmd.write_binary_pcd(map_path, map_points)

    kf_dir = tmp_path / 'keyframes'
    kf_dir.mkdir()
    pose_lines = []
    for k, (t, rot, quat) in enumerate(poses):
        cloud_world = static if k != dynamic_frame else np.vstack([static, block])
        cloud_imu = (cloud_world - t) @ rot            # p_imu = R^T (p_world - t)
        cmd.write_binary_pcd(kf_dir / f'frame_{k}.pcd', cloud_imu)
        pose_lines.append(
            'se3({:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f})'.format(
                t[0], t[1], t[2], quat[0], quat[1], quat[2], quat[3]))
    (kf_dir / 'poses.txt').write_text('\n'.join(pose_lines) + '\n')
    return map_path, kf_dir, static, block, far


def _region_masks(points):
    """按坐标把地图点分成 动态方块 / 远处无观测簇 / 静态房间 三块。"""
    inside_block = (
        (np.abs(points[:, 0] - BLOCK_CENTER_XY[0]) <= BLOCK_HALF_XY + 1e-3)
        & (np.abs(points[:, 1] - BLOCK_CENTER_XY[1]) <= BLOCK_HALF_XY + 1e-3)
        & (points[:, 2] >= BLOCK_Z[0] - 1e-3)
        & (points[:, 2] <= BLOCK_Z[1] + 1e-3))
    inside_far = points[:, 0] >= FAR_CLUSTER_OFFSET[0] - 1e-3
    return inside_block, inside_far


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------
def test_pose_parse_and_quaternion(tmp_path):
    poses_path = tmp_path / 'poses.txt'
    poses_path.write_text(
        '# comment line\n'
        'se3(1.000000,2.000000,3.000000,0.000000,0.000000,0.000000,1.000000)\n'
        'se3(4, 5, 6, 0, 0, 0.7071068, 0.7071068)\n'   # 90 度 yaw（允许空格）
    )
    trans, rot = cmd.read_poses(str(poses_path))
    assert trans.shape == (2, 3)
    assert rot.shape == (2, 3, 3)
    np.testing.assert_allclose(trans[0], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(trans[1], [4.0, 5.0, 6.0])
    np.testing.assert_allclose(rot[0], np.eye(3), atol=1e-9)
    np.testing.assert_allclose(rot[1] @ np.array([1.0, 0.0, 0.0]),
                               [0.0, 1.0, 0.0], atol=1e-6)

    bad_path = tmp_path / 'poses_bad.txt'
    bad_path.write_text('1 2 3 0 0 0 1\n')
    with pytest.raises(ValueError):
        cmd.read_poses(str(bad_path))


def test_pcd_binary_roundtrip(tmp_path):
    points = np.array([[0.0, 0.0, 0.0], [1.5, -2.25, 0.5], [3.0, 4.0, 1.0]])
    path = tmp_path / 'roundtrip.pcd'
    cmd.write_binary_pcd(path, points)
    raw = path.read_bytes()
    assert b'FIELDS x y z\n' in raw
    assert b'DATA binary\n' in raw
    read_back = pcd_reader.read_pcd_xyz(str(path))
    assert read_back.shape == (3, 3)
    np.testing.assert_allclose(read_back, points, atol=1e-6)


def test_voxel_reps_groups_and_averages():
    points = np.array([[0.0, 0.0, 0.0],
                       [0.01, 0.0, 0.0],
                       [0.2, 0.0, 0.0]])
    reps, inverse = cmd.voxel_reps(points, 0.05)
    assert reps.shape == (2, 3)
    assert inverse.tolist() == [0, 0, 1]
    np.testing.assert_allclose(reps[0], [0.005, 0.0, 0.0])
    np.testing.assert_allclose(reps[1], [0.2, 0.0, 0.0])

    reps_raw, inverse_raw = cmd.voxel_reps(points, 0.0)
    assert reps_raw.shape == (3, 3)
    assert inverse_raw.tolist() == [0, 1, 2]


def test_cli_cleans_dynamic_ghost(tmp_path):
    map_path, kf_dir, static, block, far = write_scene(tmp_path)
    output_path = tmp_path / 'clean.pcd'
    removed_path = tmp_path / 'removed.pcd'

    rc = cmd.main([
        '--map', str(map_path),
        '--keyframes', str(kf_dir),
        '--output', str(output_path),
        '--removed', str(removed_path),
        '--chunk', '5000',        # 故意开小，覆盖分块路径
    ])
    assert rc == 0
    assert output_path.is_file()
    assert removed_path.is_file()

    kept = pcd_reader.read_pcd_xyz(str(output_path))
    removed = pcd_reader.read_pcd_xyz(str(removed_path))
    total = static.shape[0] + block.shape[0] + far.shape[0]
    assert kept.shape[0] + removed.shape[0] == total

    # 按坐标区域判定：方块点被删的比例、静态房间点保留的比例。
    block_mask, far_mask = _region_masks(block)
    assert block_mask.all() and not far_mask.any()
    removed_block_mask, _ = _region_masks(removed)
    kept_block_mask, _ = _region_masks(kept)
    removed_far_mask = removed[:, 0] >= FAR_CLUSTER_OFFSET[0] - 1e-3
    kept_far_mask = kept[:, 0] >= FAR_CLUSTER_OFFSET[0] - 1e-3

    dynamic_removed_rate = removed_block_mask.sum() / float(block.shape[0])
    static_total = total - block.shape[0] - far.shape[0]
    static_removed = removed.shape[0] - removed_block_mask.sum() - removed_far_mask.sum()
    static_keep_rate = (static_total - static_removed) / float(static_total)

    print(f'dynamic removal rate = {dynamic_removed_rate:.3f}')
    print(f'static  keep    rate = {static_keep_rate:.3f}')
    assert dynamic_removed_rate >= 0.80, dynamic_removed_rate
    assert static_keep_rate >= 0.95, static_keep_rate
    # 没有证据的远处点不能删。
    assert removed_far_mask.sum() == 0
    assert kept_far_mask.sum() == far.shape[0]
    # 保留的方块点 / 删除的房间点只可能是极少数边界情况，不能成群。
    assert kept_block_mask.sum() < block.shape[0] * 0.2
    assert static_removed <= np.ceil(static_total * 0.05)


def test_missing_poses_is_rejected(tmp_path):
    map_path = tmp_path / 'map.pcd'
    cmd.write_binary_pcd(map_path, np.zeros((1, 3)))
    kf_dir = tmp_path / 'keyframes'
    kf_dir.mkdir()
    with pytest.raises(SystemExit):
        cmd.main([
            '--map', str(map_path),
            '--keyframes', str(kf_dir),
            '--output', str(tmp_path / 'out.pcd'),
        ])
