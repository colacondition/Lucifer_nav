#!/usr/bin/env python3
"""离线清理 PCD 地图里的动态残影（纯 numpy，无 open3d/scipy 依赖）。

思路（TDT clean_map_bundle 的「多帧可见空间证据」简化版）：
  1. small_glim 以 mapping.save_raw_mapping_frames:=true 建图时，每个关键帧会落盘
     frame_<idx>.pcd（IMU 系的 filtered_cloud_imu）和 poses.txt
     （每行 se3(tx,ty,tz,qx,qy,qz,qw) = T_world_imu）；
  2. 对每个关键帧，把本帧点云投成「方位角 x 俯仰角」的球形深度缓冲
     （默认 0.5 度一格，每格记最近距离；空格用邻域最小深度补洞，避免稀疏点云
     把自由空间证据漏掉）；
  3. 对地图里的每个点（按体素降采样成代表点，控制内存/耗时），逐关键帧投影到
     该帧球坐标，取同格深度 d：
        |dist - d| <= occ_tol        -> 该帧「看见」它，observed +1
        dist < d - free_margin       -> 它浮在该帧观测表面前方（自由空间），free +1
        其余（更远 = 被遮挡，或该方向无任何观测）不计；
  4. 删除规则：observed < min_observed 且 free >= min_free。
     没有证据（observed=0、free=0）的点一律保留，远处/未观测区域不会被误删。

典型动态残影（人/车走过留下的一坨点）会被大量关键帧判为「浮在自由空间里」，
而真实静态面要么在许多帧里被直接观测到，要么被别的物体挡住（不计 free）。

输出：binary PCD（FIELDS x y z），PCL 的 pcl::io::loadPCDFile<pcl::PointXYZI>
可读；另可选输出被删点 PCD 便于 CloudCompare 目检。

用法示例：
  python3 src/bringup/tools/clean_map_dynamic.py \
      --map src/bringup/PCD/RMUL.pcd \
      --keyframes /path/to/mapping_run_2026xxxx \
      --output /tmp/RMUL_clean.pcd \
      --removed /tmp/RMUL_removed.pcd
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from pcd_reader import read_pcd_xyz  # noqa: E402

# 关键帧球形深度缓冲的角分辨率（度）与补洞半径（格数）。需求里给的是 0.5 度。
BIN_DEG = 0.5
FILL_BINS = 2

# 距离小于该值的点方向不稳定，跳过证据统计（不是删除规则的一部分）。
MIN_EVIDENCE_DIST = 0.05

_SE3_RE = re.compile(r"se3\(\s*([^)]*)\)")

DEFAULT_VOXEL = 0.05
DEFAULT_OCC_TOL = 0.15
DEFAULT_FREE_MARGIN = 0.30
DEFAULT_MIN_OBSERVED = 2
DEFAULT_MIN_FREE = 3
DEFAULT_MAX_RANGE = 20.0
DEFAULT_CHUNK = 200000


# ---------------------------------------------------------------------------
# 文件 / 位姿读取
# ---------------------------------------------------------------------------
def read_poses(path):
    """解析 small_glim 的 poses.txt，返回 (trans(N,3) float64, rot(N,3,3) float64)。

    每行格式：se3(tx,ty,tz,qx,qy,qz,qw) = T_world_imu。
    """
    trans = []
    quats = []
    with open(path, 'r') as stream:
        for lineno, line in enumerate(stream, 1):
            text = line.strip()
            if not text or text.startswith('#'):
                continue
            match = _SE3_RE.search(text)
            if match is None:
                raise ValueError(
                    f'{path}:{lineno} 不是 se3(...) 格式：{text!r}')
            values = [float(v) for v in match.group(1).split(',') if v.strip()]
            if len(values) != 7:
                raise ValueError(
                    f'{path}:{lineno} se3 应有 7 个数（tx ty tz qx qy qz qw），'
                    f'实际 {len(values)} 个：{text!r}')
            trans.append(values[:3])
            quats.append(values[3:])
    if not trans:
        raise ValueError(f'{path} 里没有解析到任何 se3 位姿')
    return np.asarray(trans, dtype=np.float64), _quat_to_rot(np.asarray(quats))


def _quat_to_rot(quats):
    """(N,4) qx qy qz qw -> (N,3,3) 旋转矩阵。"""
    quats = np.asarray(quats, dtype=np.float64)
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    norms[norms == 0.0] = 1.0
    q = quats / norms
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    rot = np.empty((q.shape[0], 3, 3), dtype=np.float64)
    rot[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    rot[:, 0, 1] = 2.0 * (x * y - z * w)
    rot[:, 0, 2] = 2.0 * (x * z + y * w)
    rot[:, 1, 0] = 2.0 * (x * y + z * w)
    rot[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    rot[:, 1, 2] = 2.0 * (y * z - x * w)
    rot[:, 2, 0] = 2.0 * (x * z - y * w)
    rot[:, 2, 1] = 2.0 * (y * z + x * w)
    rot[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return rot


def find_keyframe_files(keyframes_dir):
    """返回按 index 升序的 [(idx, path), ...]（只认 frame_<idx>.pcd）。"""
    frames = []
    for name in os.listdir(keyframes_dir):
        match = re.fullmatch(r'frame_(\d+)\.pcd', name)
        if match is not None:
            frames.append((int(match.group(1)),
                           os.path.join(keyframes_dir, name)))
    frames.sort(key=lambda item: item[0])
    return frames


# ---------------------------------------------------------------------------
# PCD 写出（binary, FIELDS x y z；PCL loadPCDFile<PointXYZI> 可读）
# ---------------------------------------------------------------------------
def write_binary_pcd(path, xyz):
    """写 binary PCD。xyz 形状 (N,3)；空点云也会写出合法文件（WIDTH 0）。"""
    points = np.asarray(xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f'write_binary_pcd 需要 (N,3) 点集，收到 {points.shape}')
    points = np.ascontiguousarray(points)
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {points.shape[0]}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {points.shape[0]}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes(order='C'))


# ---------------------------------------------------------------------------
# 球形深度缓冲
# ---------------------------------------------------------------------------
def _bin_grid():
    n_az = max(1, int(round(360.0 / BIN_DEG)))
    n_el = max(1, int(round(180.0 / BIN_DEG)))
    return n_az, n_el, np.deg2rad(BIN_DEG), np.deg2rad(BIN_DEG)


def _direction_bins(points_local):
    """(N,3) 帧内坐标 -> (线性 bin 索引 (N,), 距离 (N,))。"""
    n_az, n_el, az_res, el_res = _bin_grid()
    x, y, z = points_local[:, 0], points_local[:, 1], points_local[:, 2]
    dist = np.sqrt(x * x + y * y + z * z)
    az = np.arctan2(y, x)                                      # [-pi, pi]
    el = np.arctan2(z, np.sqrt(x * x + y * y))                 # [-pi/2, pi/2]
    ia = np.floor((az + np.pi) / az_res).astype(np.int64)
    ie = np.floor((el + np.pi / 2.0) / el_res).astype(np.int64)
    np.clip(ia, 0, n_az - 1, out=ia)
    np.clip(ie, 0, n_el - 1, out=ie)
    return ia * n_el + ie, dist


def _fill_empty_bins(depth, passes):
    """球面深度图补洞：空格用邻域（切比雪夫半径 passes 格）最小深度填充。

    用「邻域最小」而不是均值/最近值，是为了让 free 判定更保守：填进来的深度
    只会更小，地图点要判 free 就必须比它更近，不会凭空产生自由空间。
    有观测的格子保持原值不动。
    """
    depth = np.asarray(depth, dtype=np.float32).copy()
    height, width = depth.shape
    for _ in range(int(passes)):
        empty = np.isinf(depth)
        if not empty.any():
            break
        padded = np.pad(depth, 1, mode='constant',
                        constant_values=np.inf)
        neighbour_min = np.full_like(depth, np.inf)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                window = padded[1 + dy:1 + dy + height, 1 + dx:1 + dx + width]
                np.fmin(neighbour_min, window, out=neighbour_min)
        depth = np.where(empty, neighbour_min, depth)
    return depth


def build_depth_buffer(frame_cloud_imu, max_range):
    """单关键帧点云（IMU 系）-> (raw 深度图, 补洞后深度图, 有效格数)。

    只统计 (0, max_range] 内的点：超出雷达量程的方向没有证据。
    """
    n_az, n_el, _, _ = _bin_grid()
    raw = np.full(n_az * n_el, np.inf, dtype=np.float32)
    points = np.asarray(frame_cloud_imu, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if points.shape[0] == 0:
        empty = raw.reshape(n_az, n_el)
        return empty, _fill_empty_bins(empty, FILL_BINS), 0
    index, dist = _direction_bins(points)
    valid = (dist > MIN_EVIDENCE_DIST) & (dist <= max_range)
    if not valid.any():
        empty = raw.reshape(n_az, n_el)
        return empty, _fill_empty_bins(empty, FILL_BINS), 0
    np.minimum.at(raw, index[valid], dist[valid].astype(np.float32))
    raw = raw.reshape(n_az, n_el)
    return raw, _fill_empty_bins(raw, FILL_BINS), int(raw.size - np.isinf(raw).sum())


# ---------------------------------------------------------------------------
# 体素降采样（只用于证据计算；输出仍保留原始点）
# ---------------------------------------------------------------------------
def voxel_reps(points, voxel):
    """把点云折叠成体素代表点（体素内均值），返回 (reps(M,3), inverse(N,))。

    inverse[i] 是原始点 i 所属代表点的下标：代表点被判删时，整个体素里的原始点
    一起删。voxel <= 0 表示不降采样（每个点一个代表点）。
    """
    points = np.asarray(points, dtype=np.float64)
    count = points.shape[0]
    if voxel is None or voxel <= 0.0:
        return points.copy(), np.arange(count, dtype=np.int64)
    ijk = np.floor(points / float(voxel)).astype(np.int64)
    ijk -= ijk.min(axis=0)
    dims = ijk.max(axis=0) + 1
    span = int(dims[0]) * int(dims[1]) * int(dims[2])
    if span >= (1 << 62):
        raise ValueError('地图坐标跨度相对于 --voxel 太大，无法打包体素键；'
                         '请增大 --voxel')
    key = (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]
    _, inverse, counts = np.unique(key, return_inverse=True, return_counts=True)
    n_rep = counts.shape[0]
    reps = np.empty((n_rep, 3), dtype=np.float64)
    for axis in range(3):
        reps[:, axis] = np.bincount(
            inverse, weights=points[:, axis], minlength=n_rep) / counts
    return reps, inverse.astype(np.int64, copy=False)


# ---------------------------------------------------------------------------
# 主清理逻辑
# ---------------------------------------------------------------------------
def clean_map(map_points, keyframe_files, poses_trans, poses_rot, *,
              voxel=DEFAULT_VOXEL, occ_tol=DEFAULT_OCC_TOL,
              free_margin=DEFAULT_FREE_MARGIN,
              min_observed=DEFAULT_MIN_OBSERVED, min_free=DEFAULT_MIN_FREE,
              max_range=DEFAULT_MAX_RANGE, chunk=DEFAULT_CHUNK,
              log=None):
    """对世界系地图点做可见性/自由空间清理。

    参数都是世界系单位（米/弧度无关，occ_tol/free_margin 用米）。
    返回 dict：keep_mask/remove_mask（作用于输入点的 bool 掩码）、
    observed/free（代表点计数）、rep_index、stats。
    """
    log = log or (lambda *_args, **_kwargs: None)
    timeout_start = time.perf_counter()

    map_points = np.asarray(map_points, dtype=np.float64)
    finite_mask = np.isfinite(map_points).all(axis=1)
    finite_points = map_points[finite_mask]
    if finite_points.shape[0] == 0:
        raise ValueError('地图里没有有限坐标的点，无法清理')

    reps, rep_index = voxel_reps(finite_points, voxel)
    n_rep = reps.shape[0]
    observed = np.zeros(n_rep, dtype=np.int32)
    free = np.zeros(n_rep, dtype=np.int32)
    chunk_size = max(1, int(chunk))

    keyframes_used = 0
    keyframes_skipped = 0
    evidence_start = time.perf_counter()
    for order, (frame_idx, frame_path) in enumerate(keyframe_files):
        if frame_idx >= poses_trans.shape[0]:
            keyframes_skipped += 1
            log(f'[warn] {os.path.basename(frame_path)} 没有对应位姿'
                f'（poses 只有 {poses_trans.shape[0]} 行），跳过')
            continue
        try:
            cloud_imu = read_pcd_xyz(frame_path)
        except (OSError, ValueError) as exc:
            keyframes_skipped += 1
            log(f'[warn] 读取关键帧失败，跳过 {frame_path}: {exc}')
            continue
        if cloud_imu.shape[0] == 0:
            keyframes_skipped += 1
            continue

        raw_depth, filled_depth, _ = build_depth_buffer(cloud_imu, max_range)
        raw_flat = raw_depth.reshape(-1)
        filled_flat = filled_depth.reshape(-1)
        trans = poses_trans[frame_idx]
        rot = poses_rot[frame_idx]

        for start in range(0, n_rep, chunk_size):
            stop = min(start + chunk_size, n_rep)
            # p_local = R^T (p_world - t)；行向量写法：v @ R
            local = (reps[start:stop] - trans) @ rot
            index, dist = _direction_bins(local)
            within_range = np.isfinite(dist) & (dist <= max_range)
            depth_occ = raw_flat[index].astype(np.float64)
            depth_free = filled_flat[index].astype(np.float64)
            occupied = (within_range & np.isfinite(depth_occ)
                        & (np.abs(dist - depth_occ) <= occ_tol))
            in_free = (within_range & np.isfinite(depth_free)
                       & (dist < depth_free - free_margin))
            if occupied.any():
                observed[start:stop] += occupied.astype(np.int32)
            if in_free.any():
                free[start:stop] += in_free.astype(np.int32)

        keyframes_used += 1
        if keyframes_used % 10 == 0:
            log(f'  已处理关键帧 {keyframes_used}/{len(keyframe_files)}')
    evidence_elapsed = time.perf_counter() - evidence_start

    remove_rep = (observed < int(min_observed)) & (free >= int(min_free))
    remove_mask = np.zeros(map_points.shape[0], dtype=bool)
    # 非有限点没有证据，永远保留
    remove_mask[np.flatnonzero(finite_mask)[remove_rep[rep_index]]] = True
    keep_mask = ~remove_mask

    elapsed = time.perf_counter() - timeout_start
    stats = {
        'input_points': int(map_points.shape[0]),
        'finite_points': int(finite_points.shape[0]),
        'reps': int(n_rep),
        'kept_points': int(keep_mask.sum()),
        'removed_points': int(remove_mask.sum()),
        'removed_ratio': (float(remove_mask.sum()) / float(map_points.shape[0])
                          if map_points.shape[0] else 0.0),
        'keyframes_total': len(keyframe_files),
        'keyframes_used': keyframes_used,
        'keyframes_skipped': keyframes_skipped,
        'observed_hist': _counts_hist(observed),
        'free_hist': _counts_hist(free),
        'evidence_elapsed': evidence_elapsed,
        'elapsed': elapsed,
        'voxel': float(voxel),
        'occ_tol': float(occ_tol),
        'free_margin': float(free_margin),
        'min_observed': int(min_observed),
        'min_free': int(min_free),
        'max_range': float(max_range),
    }
    return {
        'keep_mask': keep_mask,
        'remove_mask': remove_mask,
        'observed': observed,
        'free': free,
        'rep_index': rep_index,
        'stats': stats,
    }


def _counts_hist(counts):
    counts = np.asarray(counts)
    return {
        '0': int(np.count_nonzero(counts == 0)),
        '1': int(np.count_nonzero(counts == 1)),
        '2': int(np.count_nonzero(counts == 2)),
        '3-4': int(np.count_nonzero((counts >= 3) & (counts <= 4))),
        '5-9': int(np.count_nonzero((counts >= 5) & (counts <= 9))),
        '10+': int(np.count_nonzero(counts >= 10)),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_arg_parser():
    parser = argparse.ArgumentParser(
        description='用 small_glim 关键帧的可见性/自由空间证据离线清理 PCD 地图的'
                    '动态残影（纯 numpy，无 open3d/scipy）。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--map', required=True,
                        help='输入世界系地图 PCD（如 src/bringup/PCD/RMUL.pcd）')
    parser.add_argument('--keyframes', required=True,
                        help='关键帧目录（含 frame_<idx>.pcd 与 poses.txt）')
    parser.add_argument('--output', required=True,
                        help='清理后地图输出 PCD')
    parser.add_argument('--removed', default=None,
                        help='可选：被删点输出 PCD，便于 CloudCompare 目检')
    parser.add_argument('--voxel', type=float, default=DEFAULT_VOXEL,
                        help='证据计算用的体素降采样粒度（米，<=0 不降采样）')
    parser.add_argument('--occ-tol', type=float, default=DEFAULT_OCC_TOL,
                        help='判「观测到」的距离容差（米）')
    parser.add_argument('--free-margin', type=float, default=DEFAULT_FREE_MARGIN,
                        help='判「自由空间」时需明显近于观测表面的裕量（米）')
    parser.add_argument('--min-observed', type=int, default=DEFAULT_MIN_OBSERVED,
                        help='观测次数小于该值才可能删（保守：默认要 <2）')
    parser.add_argument('--min-free', type=int, default=DEFAULT_MIN_FREE,
                        help='自由空间证据次数达到该值才删')
    parser.add_argument('--max-range', type=float, default=DEFAULT_MAX_RANGE,
                        help='关键帧点云参与证据的最大距离（米）')
    parser.add_argument('--chunk', type=int, default=DEFAULT_CHUNK,
                        help='每批处理的地图点（代表点）数，控制内存')
    return parser


def _validate_args(args):
    if not os.path.isfile(args.map):
        raise SystemExit(f'[error] --map 不存在：{args.map}')
    if not os.path.isdir(args.keyframes):
        raise SystemExit(f'[error] --keyframes 不是目录：{args.keyframes}')
    poses_path = os.path.join(args.keyframes, 'poses.txt')
    if not os.path.isfile(poses_path):
        raise SystemExit(
            f'[error] 找不到 {poses_path}。关键帧目录需由 small_glim 在 '
            'mapping.save_raw_mapping_frames:=true 下生成。')
    if args.chunk <= 0:
        raise SystemExit('[error] --chunk 必须为正整数')
    if args.voxel < 0:
        raise SystemExit('[error] --voxel 不能为负（0 表示不降采样）')
    if args.occ_tol < 0 or args.free_margin < 0:
        raise SystemExit('[error] --occ-tol / --free-margin 不能为负')
    if args.min_observed < 0 or args.min_free < 1:
        raise SystemExit('[error] --min-observed >= 0 且 --min-free >= 1')
    if args.max_range <= 0:
        raise SystemExit('[error] --max-range 必须为正数')
    return poses_path


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    poses_path = _validate_args(args)

    t_load = time.perf_counter()
    map_points = read_pcd_xyz(args.map)
    poses_trans, poses_rot = read_poses(poses_path)
    keyframes = find_keyframe_files(args.keyframes)
    if not keyframes:
        raise SystemExit(
            f'[error] {args.keyframes} 里没有 frame_<idx>.pcd；'
            '请确认建图时开了 save_keyframes:=true 且跑完过关键帧。')
    load_elapsed = time.perf_counter() - t_load
    print(f'[clean_map_dynamic] 地图 {len(map_points)} 点：{args.map}')
    print(f'[clean_map_dynamic] 关键帧 {len(keyframes)} 个，位姿 '
          f'{poses_trans.shape[0]} 行：{args.keyframes}')

    result = clean_map(
        map_points, keyframes, poses_trans, poses_rot,
        voxel=args.voxel, occ_tol=args.occ_tol,
        free_margin=args.free_margin, min_observed=args.min_observed,
        min_free=args.min_free, max_range=args.max_range, chunk=args.chunk,
        log=print)
    stats = result['stats']

    write_binary_pcd(args.output, map_points[result['keep_mask']])
    if args.removed:
        write_binary_pcd(args.removed, map_points[result['remove_mask']])

    print(f'[clean_map_dynamic] 体素 {stats["voxel"]:g} m -> '
          f'{stats["reps"]} 个代表点（输入 {stats["input_points"]} 点）')
    print(f'[clean_map_dynamic] 保留 {stats["kept_points"]} 点，'
          f'删除 {stats["removed_points"]} 点（删除率 '
          f'{stats["removed_ratio"] * 100.0:.2f}%）')
    print(f'[clean_map_dynamic] 参数：occ_tol={stats["occ_tol"]:g} '
          f'free_margin={stats["free_margin"]:g} '
          f'min_observed={stats["min_observed"]} '
          f'min_free={stats["min_free"]} max_range={stats["max_range"]:g}')
    print(f'[clean_map_dynamic] observed 直方图（代表点）：'
          f'{stats["observed_hist"]}')
    print(f'[clean_map_dynamic] free     直方图（代表点）：'
          f'{stats["free_hist"]}')
    print(f'[clean_map_dynamic] 关键帧使用 {stats["keyframes_used"]}'
          f'/{stats["keyframes_total"]}（跳过 {stats["keyframes_skipped"]}）')
    print(f'[clean_map_dynamic] 输出：{args.output}'
          + (f'；removed：{args.removed}' if args.removed else ''))
    print(f'[clean_map_dynamic] 耗时：{stats["elapsed"]:.2f} s'
          f'（读取 {load_elapsed:.2f} s / 证据 {stats["evidence_elapsed"]:.2f} s）')
    if stats['keyframes_used'] == 0:
        print('[clean_map_dynamic] [warn] 没有成功使用任何关键帧，地图未做任何删除')
    if stats['removed_points'] == 0:
        print('[clean_map_dynamic] 提示：没有点被删除，removed PCD 是 0 点文件'
              '（PCL 不接受 0 点 PCD 属正常）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
