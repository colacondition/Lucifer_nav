#!/usr/bin/env python3
"""标定 Mid360 相对 base_link 的固定安装外参，方法 = 全向轮车原地打转。

可观测值来自车体原地旋转（全向轮自转）时录的 /lio/robo/odom bag：雷达绕底盘
旋转中心画圆，圆拟合出圆心反推水平偏移。雷达相对 base_link 的 Z 平移和固定
yaw 零位必须手填 —— 原地旋转观测不到「沿轴」和「绕轴」这两个自由度。

前提：base_link 原点 = 底盘几何中心 = 原地旋转的旋转中心。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


MIN_SAMPLE_COUNT = 10
DEFAULT_MIN_YAW_COVERAGE_DEG = 270.0
WARN_CIRCLE_RMSE_M = 0.010
WARN_HORIZONTAL_STD_M = 0.010
WARN_POSITION_Z_STD_M = 0.010
WARN_ATTITUDE_STD_RAD = math.radians(1.0)
QUATERNION_NORM_EPS = 1e-12
QUATERNION_NORM_MIN = 0.5
QUATERNION_NORM_MAX = 1.5


@dataclass
class OdometrySamples:
    """Numeric odometry samples independent of ROS message implementations."""

    timestamps: np.ndarray
    positions: np.ndarray
    orientations_xyzw: np.ndarray
    message_type: str | None = None
    frame_id: str | None = None
    child_frame_id: str | None = None


@dataclass
class CircleFit:
    """XY circle-fit result in metres."""

    center: np.ndarray
    radius: float
    rmse: float


@dataclass
class CalibrationResult:
    """Observable calibration values plus explicitly manual degrees of freedom."""

    sample_count: int
    duration_sec: float
    yaw_coverage_rad: float
    min_yaw_coverage_rad: float
    position_z_std_m: float
    circle: CircleFit
    translation: np.ndarray
    rpy_rad: np.ndarray
    translation_std: np.ndarray
    rpy_std_rad: np.ndarray
    provenance: dict[str, str]
    source_message_type: str | None = None
    source_frame_id: str | None = None
    source_child_frame_id: str | None = None


def _as_float_array(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _validated_samples(samples: OdometrySamples) -> OdometrySamples:
    timestamps = _as_float_array(samples.timestamps, "timestamps")
    positions = _as_float_array(samples.positions, "positions")
    orientations = _as_float_array(
        samples.orientations_xyzw, "orientation quaternions"
    )

    if timestamps.ndim != 1:
        raise ValueError("timestamps must have shape (N,)")
    sample_count = timestamps.shape[0]
    if sample_count < MIN_SAMPLE_COUNT:
        raise ValueError(
            f"calibration requires at least {MIN_SAMPLE_COUNT} samples; "
            f"received {sample_count}"
        )
    if positions.shape != (sample_count, 3):
        raise ValueError("positions must have shape (N, 3)")
    if orientations.shape != (sample_count, 4):
        raise ValueError("orientation quaternions must have shape (N, 4)")

    quaternion_norms = np.linalg.norm(orientations, axis=1)
    if np.any(
        (quaternion_norms < QUATERNION_NORM_MIN)
        | (quaternion_norms > QUATERNION_NORM_MAX)
    ):
        raise ValueError(
            "orientation quaternion norm is outside the plausible [0.5, 1.5] range"
        )
    orientations = orientations / quaternion_norms[:, np.newaxis]

    return OdometrySamples(
        timestamps=timestamps,
        positions=positions,
        orientations_xyzw=orientations,
        message_type=samples.message_type,
        frame_id=samples.frame_id,
        child_frame_id=samples.child_frame_id,
    )


def fit_circle(x: np.ndarray, y: np.ndarray) -> CircleFit:
    """Fit an XY circle using a linear seed followed by Gauss-Newton steps."""

    x = _as_float_array(x, "circle x coordinates").reshape(-1)
    y = _as_float_array(y, "circle y coordinates").reshape(-1)
    if x.shape != y.shape or x.size < 3:
        raise ValueError("circle fitting requires at least three paired points")

    design = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("circle points are degenerate")
    rhs = x * x + y * y
    seed, *_ = np.linalg.lstsq(design, rhs, rcond=None)
    center_x, center_y, constant = seed
    radius_squared = constant + center_x * center_x + center_y * center_y
    if radius_squared <= 0.0 or not math.isfinite(radius_squared):
        raise ValueError("circle fit produced an invalid radius")
    params = np.array([center_x, center_y, math.sqrt(radius_squared)], dtype=float)

    for _ in range(30):
        delta_x = x - params[0]
        delta_y = y - params[1]
        distances = np.hypot(delta_x, delta_y)
        if np.any(distances < 1e-12):
            raise ValueError("circle fit is ill-conditioned at the estimated center")
        residuals = distances - params[2]
        jacobian = np.column_stack(
            (-delta_x / distances, -delta_y / distances, -np.ones_like(x))
        )
        step, *_ = np.linalg.lstsq(jacobian, -residuals, rcond=None)
        params += step
        params[2] = abs(params[2])
        if np.linalg.norm(step) < 1e-12:
            break

    residuals = np.hypot(x - params[0], y - params[1]) - params[2]
    if not np.all(np.isfinite(params)) or not np.all(np.isfinite(residuals)):
        raise ValueError("circle fit produced non-finite statistics")
    if params[2] <= 0.0:
        raise ValueError("circle fit produced an invalid radius")
    return CircleFit(
        center=params[:2].copy(),
        radius=float(params[2]),
        rmse=float(np.sqrt(np.mean(residuals * residuals))),
    )


def circular_mean_std(angles: np.ndarray) -> tuple[float, float]:
    """Return circular mean and standard deviation in radians."""

    angles = _as_float_array(angles, "angles").reshape(-1)
    if angles.size == 0:
        raise ValueError("at least one angle is required")
    mean_sine = float(np.mean(np.sin(angles)))
    mean_cosine = float(np.mean(np.cos(angles)))
    mean_angle = math.atan2(mean_sine, mean_cosine)
    resultant = min(1.0, math.hypot(mean_sine, mean_cosine))
    if resultant <= 1e-15:
        return mean_angle, math.inf
    return mean_angle, math.sqrt(max(0.0, -2.0 * math.log(resultant)))


def quaternions_to_rpy(quaternions_xyzw: np.ndarray) -> np.ndarray:
    """Convert normalized ROS-order quaternions to XYZ roll, pitch, yaw."""

    quaternions = _as_float_array(
        quaternions_xyzw, "orientation quaternions"
    )
    if quaternions.ndim != 2 or quaternions.shape[1] != 4:
        raise ValueError("orientation quaternions must have shape (N, 4)")
    norms = np.linalg.norm(quaternions, axis=1)
    if np.any((norms < QUATERNION_NORM_MIN) | (norms > QUATERNION_NORM_MAX)):
        raise ValueError(
            "orientation quaternion norm is outside the plausible [0.5, 1.5] range"
        )
    quaternions = quaternions / norms[:, np.newaxis]
    x, y, z, w = quaternions.T

    roll = np.arctan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return np.column_stack((roll, pitch, yaw))


def calibrate_samples(
    samples: OdometrySamples,
    mount_z: float = 0.0,
    mount_yaw_deg: float = 0.0,
    min_yaw_coverage_deg: float = DEFAULT_MIN_YAW_COVERAGE_DEG,
) -> CalibrationResult:
    """Estimate observable mount values and combine them with manual Z/yaw."""

    samples = _validated_samples(samples)
    mount_z = float(mount_z)
    mount_yaw_rad = math.radians(float(mount_yaw_deg))
    min_yaw_coverage_deg = float(min_yaw_coverage_deg)
    if not math.isfinite(mount_z) or not math.isfinite(mount_yaw_rad):
        raise ValueError("manual mount Z and yaw must be finite")
    if not math.isfinite(min_yaw_coverage_deg) or not (
        0.0 <= min_yaw_coverage_deg <= 360.0
    ):
        raise ValueError("minimum yaw coverage must be within [0, 360] degrees")

    circle = fit_circle(samples.positions[:, 0], samples.positions[:, 1])
    rpy = quaternions_to_rpy(samples.orientations_xyzw)
    measured_yaw = rpy[:, 2]
    timestamp_order = np.argsort(samples.timestamps, kind="stable")
    unwrapped_yaw = np.unwrap(measured_yaw[timestamp_order])
    yaw_coverage_rad = float(np.max(unwrapped_yaw) - np.min(unwrapped_yaw))
    yaw_coverage_deg = math.degrees(yaw_coverage_rad)
    if yaw_coverage_deg + 1e-9 < min_yaw_coverage_deg:
        raise ValueError(
            f"yaw coverage is only {yaw_coverage_deg:.1f} deg; "
            f"at least {min_yaw_coverage_deg:.1f} deg is required for a "
            "stable circle fit. Record a broader rotation or explicitly "
            "lower --min-yaw-coverage-deg."
        )

    world_vectors = samples.positions[:, :2] - circle.center
    cos_yaw = np.cos(measured_yaw)
    sin_yaw = np.sin(measured_yaw)
    lidar_aligned_vectors = np.column_stack(
        (
            world_vectors[:, 0] * cos_yaw + world_vectors[:, 1] * sin_yaw,
            -world_vectors[:, 0] * sin_yaw + world_vectors[:, 1] * cos_yaw,
        )
    )

    cos_mount = math.cos(mount_yaw_rad)
    sin_mount = math.sin(mount_yaw_rad)
    lidar_to_base_xy = np.array(
        [[cos_mount, -sin_mount], [sin_mount, cos_mount]], dtype=float
    )
    base_vectors = lidar_aligned_vectors @ lidar_to_base_xy.T

    roll_mean, roll_std = circular_mean_std(rpy[:, 0])
    pitch_mean, pitch_std = circular_mean_std(rpy[:, 1])
    if not all(math.isfinite(value) for value in (roll_std, pitch_std)):
        raise ValueError(
            "roll/pitch dispersion is non-finite; attitude samples are not "
            "consistent enough for calibration"
        )
    translation_xy = np.mean(base_vectors, axis=0)

    duration = float(np.max(samples.timestamps) - np.min(samples.timestamps))
    return CalibrationResult(
        sample_count=samples.timestamps.size,
        duration_sec=duration,
        yaw_coverage_rad=yaw_coverage_rad,
        min_yaw_coverage_rad=math.radians(min_yaw_coverage_deg),
        position_z_std_m=float(np.std(samples.positions[:, 2])),
        circle=circle,
        translation=np.array(
            [translation_xy[0], translation_xy[1], mount_z], dtype=float
        ),
        rpy_rad=np.array([roll_mean, pitch_mean, mount_yaw_rad], dtype=float),
        translation_std=np.array(
            [np.std(base_vectors[:, 0]), np.std(base_vectors[:, 1]), 0.0],
            dtype=float,
        ),
        rpy_std_rad=np.array([roll_std, pitch_std, 0.0], dtype=float),
        provenance={
            "x": "estimated_from_bag",
            "y": "estimated_from_bag",
            "z": "manual_input",
            "roll": "estimated_from_bag",
            "pitch": "estimated_from_bag",
            "yaw": "manual_input",
        },
        source_message_type=samples.message_type,
        source_frame_id=samples.frame_id,
        source_child_frame_id=samples.child_frame_id,
    )


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll)."""

    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to XYZ roll, pitch, yaw."""

    rotation = _as_float_array(rotation, "rotation matrix")
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized ROS-order quaternion."""

    rotation = _as_float_array(rotation, "rotation matrix")
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal_index = int(np.argmax(np.diag(rotation)))
        if diagonal_index == 0:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            ) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif diagonal_index == 1:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            ) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            ) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )

    norm = float(np.linalg.norm(quaternion))
    if norm < QUATERNION_NORM_EPS:
        raise ValueError("rotation matrix produced an invalid quaternion")
    return quaternion / norm


def _axis_dict(values: np.ndarray, names: tuple[str, ...]) -> dict[str, float]:
    return {name: float(value) for name, value in zip(names, values)}


def transform_dict(
    matrix: np.ndarray,
    *,
    parent_frame: str,
    child_frame: str,
) -> dict[str, Any]:
    """Serialize one homogeneous transform into common ROS representations."""

    matrix = _as_float_array(matrix, "homogeneous transform")
    if matrix.shape != (4, 4):
        raise ValueError("homogeneous transform must have shape (4, 4)")
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    rpy_rad = matrix_to_rpy(rotation)
    quaternion = matrix_to_quaternion_xyzw(rotation)
    return {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation_m": _axis_dict(translation, ("x", "y", "z")),
        "translation_mm": _axis_dict(translation * 1000.0, ("x", "y", "z")),
        "rpy_rad": _axis_dict(rpy_rad, ("roll", "pitch", "yaw")),
        "rpy_deg": _axis_dict(np.rad2deg(rpy_rad), ("roll", "pitch", "yaw")),
        "quaternion_xyzw": _axis_dict(quaternion, ("x", "y", "z", "w")),
        "rotation_matrix_3x3": rotation.tolist(),
        "matrix_4x4": matrix.tolist(),
    }


def build_report(
    result: CalibrationResult,
    bag_path: str | Path,
    topic: str,
) -> dict[str, Any]:
    """Build a complete, plain-Python calibration report."""

    _as_float_array(result.circle.center, "circle center")
    _as_float_array(result.translation, "calibrated translation")
    _as_float_array(result.rpy_rad, "calibrated RPY")
    _as_float_array(result.translation_std, "translation dispersion")
    _as_float_array(result.rpy_std_rad, "attitude dispersion")
    scalar_statistics = np.array(
        [
            result.duration_sec,
            result.yaw_coverage_rad,
            result.min_yaw_coverage_rad,
            result.position_z_std_m,
            result.circle.radius,
            result.circle.rmse,
        ],
        dtype=float,
    )
    _as_float_array(scalar_statistics, "calibration statistics")

    rotation = rpy_to_matrix(*result.rpy_rad)
    forward = np.eye(4)
    forward[:3, :3] = rotation
    forward[:3, 3] = result.translation

    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ result.translation)

    warnings: list[str] = []
    min_yaw_coverage_deg = math.degrees(result.min_yaw_coverage_rad)
    if min_yaw_coverage_deg + 1e-9 < DEFAULT_MIN_YAW_COVERAGE_DEG:
        warnings.append(
            f"The minimum yaw coverage was explicitly lowered to "
            f"{min_yaw_coverage_deg:.1f} deg from the recommended "
            f"{DEFAULT_MIN_YAW_COVERAGE_DEG:.1f} deg. Treat this result as "
            "lower confidence and inspect the fit diagnostics carefully."
        )
    if result.circle.rmse > WARN_CIRCLE_RMSE_M:
        warnings.append(
            f"Data quality warning: circle-fit RMSE is "
            f"{result.circle.rmse * 1000.0:.2f} mm, above the "
            f"{WARN_CIRCLE_RMSE_M * 1000.0:.2f} mm warning threshold. "
            "Check that the chassis stayed still and LIO did not drift."
        )
    horizontal_std = float(np.max(result.translation_std[:2]))
    if horizontal_std > WARN_HORIZONTAL_STD_M:
        warnings.append(
            f"Data quality warning: horizontal translation dispersion reaches "
            f"{horizontal_std * 1000.0:.2f} mm, above the "
            f"{WARN_HORIZONTAL_STD_M * 1000.0:.2f} mm warning threshold."
        )
    attitude_std = float(np.max(result.rpy_std_rad[:2]))
    if attitude_std > WARN_ATTITUDE_STD_RAD:
        warnings.append(
            f"Data quality warning: roll/pitch attitude dispersion reaches "
            f"{math.degrees(attitude_std):.2f} deg, above the "
            f"{math.degrees(WARN_ATTITUDE_STD_RAD):.2f} deg warning threshold."
        )
    if result.position_z_std_m > WARN_POSITION_Z_STD_M:
        warnings.append(
            f"Data quality warning: odometry Z dispersion is "
            f"{result.position_z_std_m * 1000.0:.2f} mm, above the "
            f"{WARN_POSITION_Z_STD_M * 1000.0:.2f} mm warning threshold."
        )
    forward_transform = transform_dict(
        forward,
        parent_frame="base_link",
        child_frame="livox_frame",
    )
    inverse_transform = transform_dict(
        inverse,
        parent_frame="livox_frame",
        child_frame="base_link",
    )
    rpy = forward_transform["rpy_rad"]
    translation = forward_transform["translation_m"]
    xyz_text = f'{translation["x"]:.9f} {translation["y"]:.9f} {translation["z"]:.9f}'
    rpy_text = f'{rpy["roll"]:.9f} {rpy["pitch"]:.9f} {rpy["yaw"]:.9f}'

    return {
        "format_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": {
            "bag_path": str(bag_path),
            "topic": str(topic),
            "sample_count": int(result.sample_count),
            "duration_sec": float(result.duration_sec),
            "message_type": result.source_message_type,
            "frame_id": result.source_frame_id,
            "child_frame_id": result.source_child_frame_id,
        },
        "assumptions": [
            (
                "The odometry pose is a LiDAR pose (odom -> base_link carrying "
                "the lidar pose). small_glim already applies sensors.T_lidar_imu "
                "when publishing /lio/robo/odom, so this bag is valid for the "
                "mechanical calibration."
            ),
            (
                "The chassis rotates in place on a level floor (omnidirectional "
                "wheels) in a static environment, and the base_link origin is "
                "the rotation center."
            ),
            (
                "mount-z is the base_link -> livox_frame Z translation; "
                "roll/pitch are the lidar mounting tilt measured relative to "
                "the level chassis."
            ),
        ],
        "provenance": dict(result.provenance),
        "quality": {
            "circle_center_odom_m": _axis_dict(result.circle.center, ("x", "y")),
            "circle_radius_m": float(result.circle.radius),
            "circle_radius_mm": float(result.circle.radius * 1000.0),
            "circle_fit_rmse_m": float(result.circle.rmse),
            "circle_fit_rmse_mm": float(result.circle.rmse * 1000.0),
            "yaw_coverage_rad": float(result.yaw_coverage_rad),
            "yaw_coverage_deg": float(math.degrees(result.yaw_coverage_rad)),
            "minimum_yaw_coverage_rad": float(result.min_yaw_coverage_rad),
            "minimum_yaw_coverage_deg": float(min_yaw_coverage_deg),
            "position_z_std_m": float(result.position_z_std_m),
            "position_z_std_mm": float(result.position_z_std_m * 1000.0),
            "warning_thresholds": {
                "circle_fit_rmse_m": WARN_CIRCLE_RMSE_M,
                "horizontal_translation_std_m": WARN_HORIZONTAL_STD_M,
                "position_z_std_m": WARN_POSITION_Z_STD_M,
                "roll_pitch_std_rad": WARN_ATTITUDE_STD_RAD,
                "roll_pitch_std_deg": math.degrees(WARN_ATTITUDE_STD_RAD),
            },
            "translation_std_m": _axis_dict(
                result.translation_std, ("x", "y", "z")
            ),
            "rpy_std_rad": _axis_dict(
                result.rpy_std_rad, ("roll", "pitch", "yaw")
            ),
            "rpy_std_deg": _axis_dict(
                np.rad2deg(result.rpy_std_rad), ("roll", "pitch", "yaw")
            ),
        },
        "transforms": {
            "base_link_to_livox_frame": forward_transform,
            "livox_frame_to_base_link": inverse_transform,
        },
        "configuration": {
            "fixed_mount_xacro_origin": (
                f'<origin xyz="{xyz_text}" rpy="{rpy_text}" />'
            ),
            "separated_yaml": {
                "base_link2livox_frame": {
                    # 值带引号，可直接粘进 config/reality/measurement_params_real.yaml
                    # （real.launch.py 把这些字符串原样拼进 xacro 命令，xacro 需要引号）。
                    "xyz": f'"{xyz_text}"',
                    "rpy": f'"{rpy_text}"',
                }
            },
            "do_not_replace": "small_glim sensors.T_lidar_imu",
        },
        "inspection": {
            "tf2_echo": (
                "ros2 run tf2_ros tf2_echo base_link livox_frame"
            )
        },
        "warnings": warnings,
    }


def write_reports(
    report: dict[str, Any], output_prefix: str | Path
) -> tuple[Path, Path]:
    """Write equivalent UTF-8 YAML and JSON reports."""

    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - depends on robot image
        raise RuntimeError(
            "PyYAML is required to write the YAML result: install python3-yaml"
        ) from exc

    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    yaml_path = Path(f"{prefix}.yaml")
    json_path = Path(f"{prefix}.json")
    yaml_path.write_text(
        yaml.safe_dump(report, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return yaml_path, json_path


def read_rosbag(bag_path: str | Path, topic: str) -> OdometrySamples:
    """Read nav_msgs/Odometry-like samples from a ROS 2 bag.

    ``rosbags`` is imported lazily so importing this module and displaying
    ``--help`` continue to work on machines that only consume saved reports.
    """

    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:  # pragma: no cover - depends on robot image
        raise RuntimeError(
            "Reading ROS 2 bags requires the 'rosbags' Python package. "
            "Install it with: python3 -m pip install rosbags"
        ) from exc

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise ValueError(f"ROS bag path does not exist: {bag_path}")
    if not topic:
        raise ValueError("odometry topic must not be empty")

    timestamps: list[float] = []
    positions: list[list[float]] = []
    orientations: list[list[float]] = []
    frame_ids: set[str] = set()
    child_frame_ids: set[str] = set()
    expected_message_type = "nav_msgs/msg/Odometry"

    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with Reader(bag_path) as reader:
            selected_connections = [
                connection
                for connection in reader.connections
                if connection.topic == topic
            ]
            if not selected_connections:
                available = sorted(
                    {connection.topic for connection in reader.connections}
                )
                available_text = ", ".join(available) if available else "<none>"
                raise ValueError(
                    f"topic '{topic}' was not found in {bag_path}; "
                    f"available topics: {available_text}"
                )
            message_types = {connection.msgtype for connection in selected_connections}
            if message_types != {expected_message_type}:
                found = ", ".join(sorted(message_types))
                raise ValueError(
                    f"topic '{topic}' must use {expected_message_type}; found: {found}"
                )

            for connection, timestamp, rawdata in reader.messages(
                connections=selected_connections
            ):
                message = typestore.deserialize_cdr(rawdata, connection.msgtype)
                try:
                    pose = message.pose.pose
                    position = pose.position
                    orientation = pose.orientation
                    frame_id = str(message.header.frame_id)
                    child_frame_id = str(message.child_frame_id)
                except AttributeError as exc:
                    raise ValueError(
                        f"topic '{topic}' message type '{connection.msgtype}' "
                        "does not provide the required Odometry fields"
                    ) from exc

                if not frame_id:
                    raise ValueError(
                        f"topic '{topic}' contains an empty header.frame_id"
                    )
                if not child_frame_id:
                    raise ValueError(
                        f"topic '{topic}' contains an empty child_frame_id"
                    )
                frame_ids.add(frame_id)
                child_frame_ids.add(child_frame_id)

                timestamps.append(float(timestamp) * 1e-9)
                positions.append(
                    [float(position.x), float(position.y), float(position.z)]
                )
                orientations.append(
                    [
                        float(orientation.x),
                        float(orientation.y),
                        float(orientation.z),
                        float(orientation.w),
                    ]
                )
    except ValueError:
        raise
    except Exception as exc:
        raise RuntimeError(f"failed to read ROS bag '{bag_path}': {exc}") from exc

    if not timestamps:
        raise ValueError(f"topic '{topic}' contains no messages")
    if len(frame_ids) != 1:
        raise ValueError(
            f"topic '{topic}' changes header.frame_id within the bag: "
            f"{', '.join(sorted(frame_ids))}"
        )
    if len(child_frame_ids) != 1:
        raise ValueError(
            f"topic '{topic}' changes child_frame_id within the bag: "
            f"{', '.join(sorted(child_frame_ids))}"
        )
    child_frame_id = next(iter(child_frame_ids))
    if child_frame_id != "base_link":
        raise ValueError(
            f"topic '{topic}' child_frame_id is '{child_frame_id}', but this "
            "tool requires the LiDAR pose child_frame_id 'base_link' "
            "(small_glim publishes the lidar pose as base_link)"
        )

    return OdometrySamples(
        timestamps=np.asarray(timestamps, dtype=float),
        positions=np.asarray(positions, dtype=float),
        orientations_xyzw=np.asarray(orientations, dtype=float),
        message_type=expected_message_type,
        frame_id=next(iter(frame_ids)),
        child_frame_id=child_frame_id,
    )


def save_plot(
    samples: OdometrySamples,
    result: CalibrationResult,
    output_prefix: str | Path,
) -> Path:
    """Save an XY fit and attitude diagnostic plot without opening a GUI."""

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - depends on robot image
        raise RuntimeError(
            "Plot output requires matplotlib. Install python3-matplotlib, "
            "or run with --no-plot."
        ) from exc

    samples = _validated_samples(samples)
    rpy = quaternions_to_rpy(samples.orientations_xyzw)
    relative_time = samples.timestamps - samples.timestamps[0]
    circle_angle = np.linspace(0.0, 2.0 * math.pi, 361)
    circle_x = result.circle.center[0] + result.circle.radius * np.cos(circle_angle)
    circle_y = result.circle.center[1] + result.circle.radius * np.sin(circle_angle)

    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    axes[0].plot(
        samples.positions[:, 0],
        samples.positions[:, 1],
        ".",
        markersize=2,
        label="odometry samples",
    )
    axes[0].plot(circle_x, circle_y, "-", linewidth=1.5, label="fitted circle")
    axes[0].plot(
        result.circle.center[0],
        result.circle.center[1],
        "+",
        markersize=12,
        markeredgewidth=2,
        label="rotation center",
    )
    axes[0].set_title("LiDAR XY trajectory")
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(relative_time, np.rad2deg(rpy[:, 0]), label="roll")
    axes[1].plot(relative_time, np.rad2deg(rpy[:, 1]), label="pitch")
    axes[1].axhline(
        math.degrees(result.rpy_rad[0]),
        color="C0",
        linestyle="--",
        alpha=0.6,
    )
    axes[1].axhline(
        math.degrees(result.rpy_rad[1]),
        color="C1",
        linestyle="--",
        alpha=0.6,
    )
    axes[1].set_title("Mount attitude consistency")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Angle [deg]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    plot_path = Path(f"{Path(output_prefix)}.png")
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    return plot_path


def print_summary(
    result: CalibrationResult,
    report: dict[str, Any],
    yaml_path: Path,
    json_path: Path,
    plot_path: Path | None,
) -> None:
    """Print the complete report in a readable terminal-oriented form."""

    source = report["source"]
    quality = report["quality"]
    print("\n标定完成：LiDAR–底盘固定外参（base_link → livox_frame）")
    print(f"生成时间: {report['generated_at_utc']}")
    print(
        f"来源: bag={source['bag_path']}, topic={source['topic']}, "
        f"type={source['message_type']}, frame={source['frame_id']} -> "
        f"{source['child_frame_id']}"
    )
    print(
        f"样本: count={source['sample_count']}, "
        f"duration={source['duration_sec']:.3f} s"
    )
    print("provenance: " + json.dumps(report["provenance"], ensure_ascii=False))
    print(
        f"  圆拟合: radius={result.circle.radius:.6f} m, "
        f"RMSE={result.circle.rmse * 1000.0:.3f} mm"
    )
    print(
        f"  Yaw 覆盖: {quality['yaw_coverage_deg']:.2f} deg "
        f"(required {quality['minimum_yaw_coverage_deg']:.2f} deg)"
    )
    print(
        f"  Z/std: {quality['position_z_std_mm']:.3f} mm, "
        "translation std [m]: "
        + json.dumps(quality["translation_std_m"], ensure_ascii=False)
    )
    print(
        "  RPY std [deg]: "
        + json.dumps(quality["rpy_std_deg"], ensure_ascii=False)
    )

    for transform_key in (
        "base_link_to_livox_frame",
        "livox_frame_to_base_link",
    ):
        transform = report["transforms"][transform_key]
        print(
            f"\n{transform['parent_frame']} -> {transform['child_frame']}"
        )
        print(
            "  XYZ [m]: "
            + json.dumps(transform["translation_m"], ensure_ascii=False)
        )
        print(
            "  XYZ [mm]: "
            + json.dumps(transform["translation_mm"], ensure_ascii=False)
        )
        print(
            "  RPY [rad]: "
            + json.dumps(transform["rpy_rad"], ensure_ascii=False)
        )
        print(
            "  RPY [deg]: "
            + json.dumps(transform["rpy_deg"], ensure_ascii=False)
        )
        print(
            "  Quaternion (x,y,z,w): "
            + json.dumps(transform["quaternion_xyzw"], ensure_ascii=False)
        )
        print("  Rotation 3x3:")
        for row in transform["rotation_matrix_3x3"]:
            print("    " + " ".join(f"{value: .9f}" for value in row))
        print("  Homogeneous transform 4x4:")
        for row in transform["matrix_4x4"]:
            print("    " + " ".join(f"{value: .9f}" for value in row))

    print("\n配置建议:")
    print(f"  Xacro: {report['configuration']['fixed_mount_xacro_origin']}")
    print(
        "  YAML: "
        + json.dumps(
            report["configuration"]["separated_yaml"], ensure_ascii=False
        )
    )
    print(f"  TF 检查: {report['inspection']['tf2_echo']}")
    print(f"  YAML: {yaml_path}")
    print(f"  JSON: {json_path}")
    if plot_path is not None:
        print(f"  图表: {plot_path}")
    print("  注意: 该结果不是 small_glim 的 sensors.T_lidar_imu，不要覆盖内部 IMU 外参。")
    print(
        "  前提: /lio/robo/odom 是 small_glim 发布的 LiDAR 位姿（child=base_link）；"
        "标定时车体原地打转，base_link 原点 = 旋转中心。"
    )


def build_argument_parser() -> argparse.ArgumentParser:
    """Create the command-line parser with this navigation stack's defaults."""

    parser = argparse.ArgumentParser(
        description=(
            "从全向轮车原地打转时的 /lio/robo/odom 标定固定外参 "
            "base_link -> livox_frame，并输出完整 TF 信息。"
        )
    )
    parser.add_argument("bag_path", type=Path, help="ROS 2 bag 目录")
    parser.add_argument(
        "--topic",
        default="/lio/robo/odom",
        help="Odometry 话题（默认: /lio/robo/odom）",
    )
    parser.add_argument(
        "--mount-z",
        type=float,
        default=0.175,
        help=(
            "base_link 到 livox_frame 的 Z 平移，单位 m；"
            "雷达离地高度的唯一来源。默认 0.175 是仿真值"
            "（config/simulation/measurement_params_sim.yaml）；"
            "实车跑请传 --mount-z 0.49"
        ),
    )
    parser.add_argument(
        "--mount-yaw-deg",
        type=float,
        default=0.0,
        help="固定安装 yaw 零位，单位 deg（默认: 0）",
    )
    parser.add_argument(
        "--min-yaw-coverage-deg",
        type=float,
        default=DEFAULT_MIN_YAW_COVERAGE_DEG,
        help=(
            "圆拟合所需的最小 yaw 覆盖角；机械限位时可显式降低 "
            "（默认: 270）"
        ),
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=Path("lidar_extrinsic_result"),
        help="输出文件前缀（默认: lidar_extrinsic_result）",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="不生成 PNG 诊断图（可避免 matplotlib 依赖）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run bag loading, calibration and report generation."""

    args = build_argument_parser().parse_args(argv)
    try:
        samples = read_rosbag(args.bag_path, args.topic)
        result = calibrate_samples(
            samples,
            mount_z=args.mount_z,
            mount_yaw_deg=args.mount_yaw_deg,
            min_yaw_coverage_deg=args.min_yaw_coverage_deg,
        )
        report = build_report(result, bag_path=args.bag_path, topic=args.topic)
        for warning in report["warnings"]:
            print(f"警告: {warning}", file=sys.stderr)
        plot_path = None
        if not args.no_plot:
            plot_path = save_plot(samples, result, args.output_prefix)
        yaml_path, json_path = write_reports(report, args.output_prefix)
        print_summary(result, report, yaml_path, json_path, plot_path)
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
