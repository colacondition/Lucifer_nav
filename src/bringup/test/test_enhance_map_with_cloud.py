import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import enhance_map_with_cloud as e  # noqa: E402


def test_project_counts_band_and_bounds():
    points = np.array([
        [0.0, 0.0, 0.10],   # in band -> (0,0)
        [0.0, 0.0, 0.12],   # in band -> (0,0) 第二次
        [0.1, 0.0, 0.05],   # in band -> (2,0)
        [0.0, 0.0, 5.0],    # 高于 band
        [-10.0, 0.0, 0.1],  # 出界
    ])
    counts = e.project_points_to_counts(
        points, origin_x=0.0, origin_y=0.0, resolution=0.05,
        width=4, height=2, ground_z=0.0, min_height=0.03, max_height=0.6)
    assert counts[0, 0] == 2
    assert counts[0, 2] == 1
    assert counts.sum() == 3
    assert e.project_points_to_mask(
        points, 0.0, 0.0, 0.05, 4, 2, 0.0, 0.03, 0.6, min_points=2)[0, 0]


def test_project_range_limit_and_empty():
    points = np.array([[10.0, 0.0, 0.1]])
    counts = e.project_points_to_counts(
        points, 0.0, 0.0, 0.05, 400, 10, 0.0, 0.03, 0.6, max_range=2.0)
    assert counts.sum() == 0
    assert e.project_points_to_counts(
        np.empty((0, 3)), 0.0, 0.0, 0.05, 4, 4, 0.0, 0.0, 1.0).shape == (4, 4)


def test_dilate_mask_adds_four_neighbors():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    out = e.dilate_mask(mask, 1)
    assert out.sum() == 5
    assert out[1, 2] and out[3, 2] and out[2, 1] and out[2, 3]
    assert e.dilate_mask(mask, 0).sum() == 1


def test_merge_grid_preserves_three_states_and_flips_rows():
    data = np.array([
        [100, 0],   # y=0 行：占据、空闲
        [0, -1],    # y=1 行：空闲、未知
    ], dtype=np.int16)
    mask = np.zeros((2, 2), dtype=bool)
    mask[0, 1] = True  # 给 y=0 的空闲格加障碍
    image = e.merge_grid_to_image(data, mask)
    # PGM 第 0 行是 y 最大（原 data 第 1 行），且被翻转
    assert image[0, 0] == e.PIXEL_FREE
    assert image[0, 1] == e.PIXEL_UNKNOWN
    assert image[1, 0] == e.PIXEL_OCCUPIED
    assert image[1, 1] == e.PIXEL_OCCUPIED  # mask 覆盖了原空闲
