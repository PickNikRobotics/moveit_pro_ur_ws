# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Pre- and post-processing for the merged SAM3 QNN graph. NumPy and OpenCV only: no ROS, no NPU.

Matches what MoveIt Pro's GetMasks2DFromExemplar does with its SAM3 bundle (moveit_pro_ml SAM3Detect):
the image is stretched to the model's square input and normalized with mean and std 0.5; a detection's
score is sigmoid(logit) times sigmoid(presence) and is kept at or above the threshold, in decoder order
with no NMS; its mask is the decoder's logits at or above zero, upscaled nearest to the image, 0 or 255.
"""

import re

import cv2
import numpy as np

INPUT_SIZE = 1008
TOKEN_COUNT = 32


def logical_name(tensor_name: str) -> str:
    """The merged graph prefixes each tensor with its stage (s1_image, s3_pred_masks); drop that prefix."""
    return re.sub(r"^s\d+_", "", tensor_name)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.asarray(x, dtype=np.float64)))


def image_to_rgb(encoding: str, height: int, width: int, step: int, data: bytes) -> np.ndarray:
    """Decode a sensor_msgs/Image buffer into an HxWx3 uint8 RGB array."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding '{encoding}'; send rgb8, bgr8, rgba8 or bgra8")
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError(f"image size {width}x{height} with step {step} is invalid")
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < step * height:
        raise ValueError(f"image data has {buf.size} bytes, expected {step * height}")
    img = buf[: step * height].reshape(height, step)[:, : width * channels].reshape(height, width, channels)
    if encoding.startswith("bgr"):
        img = img[..., [2, 1, 0]]
    return np.ascontiguousarray(img[..., :3])


def preprocess(rgb: np.ndarray) -> np.ndarray:
    """Stretch to the model size (bilinear), scale to [-1, 1], lay out as 1x3xHxW float32."""
    resized = cv2.resize(rgb, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    scaled = resized / 255.0 * 2.0 - 1.0
    return np.ascontiguousarray(scaled.transpose(2, 0, 1)[np.newaxis])


def select_detections(pred_logits, presence_logit, pred_masks, threshold: float, height: int, width: int):
    """Detections scoring at or above the threshold, in decoder order, as (query, score, HxW uint8 0/255 mask)."""
    if not np.isfinite(threshold):
        raise ValueError("confidence threshold must be finite")
    logits = np.asarray(pred_logits, dtype=np.float64).reshape(-1)
    masks = np.asarray(pred_masks, dtype=np.float32)
    masks = masks.reshape(-1, masks.shape[-2], masks.shape[-1])
    if masks.shape[0] != logits.size:
        raise ValueError(f"{logits.size} scores but {masks.shape[0]} masks")
    presence = float(sigmoid(np.asarray(presence_logit, dtype=np.float64).reshape(-1)[0]))
    scores = sigmoid(logits) * presence
    out = []
    for i, score in enumerate(scores):
        if not score >= threshold:  # written so a NaN score is dropped
            continue
        binary = np.where(masks[i] >= 0.0, np.uint8(255), np.uint8(0))
        out.append((i, float(score), cv2.resize(binary, (width, height), interpolation=cv2.INTER_NEAREST)))
    return out
