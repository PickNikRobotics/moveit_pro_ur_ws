# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
import math

import numpy as np
import pytest

from sam3_npu_server import core


def test_logical_name_drops_the_stage_prefix():
    assert core.logical_name("s1_image") == "image"
    assert core.logical_name("s3_presence_logit_dec") == "presence_logit_dec"
    assert core.logical_name("pred_masks") == "pred_masks"


def test_image_to_rgb_swaps_bgr_and_drops_alpha_and_row_padding():
    # 1x2 bgra8 image with 4 bytes of row padding.
    data = bytes([1, 2, 3, 9, 4, 5, 6, 9, 0, 0, 0, 0])
    rgb = core.image_to_rgb("bgra8", 1, 2, 12, data)
    assert rgb.shape == (1, 2, 3)
    assert rgb.tolist() == [[[3, 2, 1], [6, 5, 4]]]


def test_image_to_rgb_rejects_unsupported_or_short_images():
    with pytest.raises(ValueError):
        core.image_to_rgb("mono8", 1, 1, 1, b"\x00")
    with pytest.raises(ValueError):
        core.image_to_rgb("rgb8", 2, 2, 6, b"\x00" * 6)


def test_preprocess_is_square_nchw_in_minus_one_to_one():
    rgb = np.zeros((480, 640, 3), np.uint8)
    rgb[..., 0] = 255
    x = core.preprocess(rgb)
    assert x.shape == (1, 3, core.INPUT_SIZE, core.INPUT_SIZE)
    assert x.dtype == np.float32
    assert np.allclose(x[0, 0], 1.0) and np.allclose(x[0, 1:], -1.0)


def test_select_detections_scores_with_presence_and_keeps_decoder_order():
    logits = np.array([3.0, -5.0, 2.0])
    presence = np.array([[4.0]])
    masks = np.full((3, 4, 4), -1.0, np.float32)
    masks[0, :2, :2] = 1.0
    masks[2, 2:, 2:] = 0.0  # a logit of exactly zero is inside, as in Pro
    kept = core.select_detections(logits, presence, masks, threshold=0.5, height=8, width=8)
    assert [i for i, _, _ in kept] == [0, 2]
    expected = 1 / (1 + math.exp(-3.0)) * 1 / (1 + math.exp(-4.0))
    assert kept[0][1] == pytest.approx(expected)
    mask0 = kept[0][2]
    assert mask0.shape == (8, 8) and mask0.dtype == np.uint8
    assert mask0[:4, :4].min() == 255 and mask0[4:, :].max() == 0
    assert kept[1][2][4:, 4:].min() == 255


def test_select_detections_low_presence_drops_everything():
    kept = core.select_detections(np.array([10.0]), np.array([-10.0]), np.ones((1, 2, 2)), 0.5, 2, 2)
    assert kept == []


def test_select_detections_rejects_mismatched_counts_and_nan_threshold():
    with pytest.raises(ValueError):
        core.select_detections(np.zeros(2), np.zeros(1), np.zeros((3, 2, 2)), 0.5, 2, 2)
    with pytest.raises(ValueError):
        core.select_detections(np.zeros(1), np.zeros(1), np.zeros((1, 2, 2)), float("nan"), 2, 2)
