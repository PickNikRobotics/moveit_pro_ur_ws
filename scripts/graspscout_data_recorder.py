#!/usr/bin/env python3
"""Record a GraspScout fine-tuning sample for every pick the Unshelve Product Loop plans.

The loop publishes a pick record on /unshelve/pick_record (std_msgs/String, "key=value;..."):

  event=planned    the pick is planned and the arm is still at the view the detector saw. This
                   node takes its own wrist-camera frame then (colour, depth aligned to colour,
                   intrinsics, the camera pose from TF) and writes a sample with the product,
                   the detector's (Gemini's) point and the planned grasp, projected into the image.
  event=saved      the pick's Trainer episode was kept: the sample is marked saved, with the
                   Trainer recording id that links it to the episode.
  event=discarded  the pick failed: the sample is kept, marked discarded (a negative example).

A sample whose episode is later deleted in the Trainer is dropped by export_graspscout_data.py,
which checks the recording still exists, so only grasps you kept become training data.

Each sample is a directory  <out>/<dataset>/<time>_run<N>_<product>/  with
  rgb.png      the wrist colour image (8-bit, RGB order on disk is the usual PNG/BGR handling)
  depth.png    depth aligned to colour, uint16 millimetres (0 = no depth)
  record.json  everything else (see SAMPLE_KEYS below)

Run inside the Runtime container as the host user, or from the host with
scripts/graspscout_data_recorder.sh. Long-running; Ctrl-C stops it. Start it before a collection.
"""

import argparse
import datetime as dt
import json
import math
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

RECORD_TOPIC = "/unshelve/pick_record"
COLOR_TOPIC = "/camera/wrist_mounted_camera/color/image_raw"
DEPTH_TOPIC = "/camera/wrist_mounted_camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/wrist_mounted_camera/color/camera_info"
DEFAULT_OUT = "~/.local/share/moveit_pro/graspscout_data"
MAX_FRAME_AGE_S = 1.0
GRIPPER = "robotiq_2f85"


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def parse_record(text):
    out = {}
    for part in text.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()
    if msg.encoding in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if msg.encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    raise ValueError(f"unsupported colour encoding {msg.encoding}")


def depth_to_mm(msg):
    if msg.encoding in ("16UC1", "mono16"):
        d = np.frombuffer(msg.data, dtype="<u2" if not msg.is_bigendian else ">u2")
        return d.reshape(msg.height, msg.step // 2)[:, : msg.width].astype(np.uint16)
    if msg.encoding == "32FC1":
        d = np.frombuffer(msg.data, dtype="<f4" if not msg.is_bigendian else ">f4")
        d = d.reshape(msg.height, msg.step // 4)[:, : msg.width]
        return np.nan_to_num(d * 1000.0, nan=0.0, posinf=0.0, neginf=0.0).clip(0, 65535).astype(np.uint16)
    raise ValueError(f"unsupported depth encoding {msg.encoding}")


class Recorder(Node):
    def __init__(self, out_root):
        super().__init__("graspscout_data_recorder")
        self.out_root = os.path.expanduser(out_root)
        self.lock = threading.Lock()
        self.color = self.depth = self.info = None
        self.current = None  # (sample_dir, record dict) of the last planned pick
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Image, COLOR_TOPIC, lambda m: self._keep("color", m), qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, lambda m: self._keep("depth", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO_TOPIC, lambda m: self._keep("info", m), qos_profile_sensor_data)
        self.create_subscription(String, RECORD_TOPIC, self._on_record,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f"recording GraspScout samples to {self.out_root} on every {RECORD_TOPIC} event=planned")

    def _keep(self, key, msg):
        with self.lock:
            setattr(self, key, msg)

    def _age(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_record(self, msg):
        rec = parse_record(msg.data)
        event = rec.get("event")
        try:
            if event == "planned":
                self._planned(rec)
            elif event in ("saved", "discarded"):
                self._finish(rec, event)
        except Exception as e:  # keep recording the next pick whatever went wrong with this one
            self.get_logger().error(f"pick record {event}: {e}")

    def _planned(self, rec):
        if self.current is not None:
            self._update(status="abandoned")  # planned again with no saved/discarded in between
        with self.lock:
            color, depth, info = self.color, self.depth, self.info
        if color is None or depth is None or info is None:
            raise RuntimeError("no wrist colour, depth or camera_info received yet; is the camera up?")
        for name, m in (("colour", color), ("depth", depth)):
            if self._age(m) > MAX_FRAME_AGE_S:
                raise RuntimeError(f"latest wrist {name} frame is {self._age(m):.1f} s old; not recording this pick")
        bgr, depth_mm = image_to_bgr(color), depth_to_mm(depth)
        if depth_mm.shape[:2] != bgr.shape[:2]:
            raise RuntimeError(f"depth {depth_mm.shape[:2]} is not aligned to colour {bgr.shape[:2]}")
        frame = color.header.frame_id
        t = self.tf.lookup_transform("world", frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)).transform
        R_wc = quat_to_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        p_wc = np.array([t.translation.x, t.translation.y, t.translation.z])
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        H, W = bgr.shape[:2]
        up_cam = (R_wc.T @ np.array([0.0, 0.0, 1.0])).tolist()  # world +Z in the camera optical frame
        g_world = np.array([float(rec["gx"]), float(rec["gy"]), float(rec["gz"])])
        g_cam = R_wc.T @ (g_world - p_wc)
        R_wg = quat_to_matrix(float(rec["qx"]), float(rec["qy"]), float(rec["qz"]), float(rec["qw"]))
        approach_cam = (R_wc.T @ R_wg[:, 2]).tolist()   # grasp_link +Z (the approach), camera frame
        fingers_cam = (R_wc.T @ R_wg[:, 0]).tolist()    # grasp_link +X (the finger axis), camera frame
        grasp_px = [fx * g_cam[0] / g_cam[2] + cx, fy * g_cam[1] / g_cam[2] + cy] if g_cam[2] > 1e-6 else None
        u, v = float(rec["u"]), float(rec["v"])
        product = rec.get("product", "unknown")
        dataset = rec.get("dataset", "unknown")
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        sample_dir = os.path.join(self.out_root, dataset, f"{stamp}_run{rec.get('run', '0')}_{product}")
        os.makedirs(sample_dir, exist_ok=True)
        cv2.imwrite(os.path.join(sample_dir, "rgb.png"), bgr)
        cv2.imwrite(os.path.join(sample_dir, "depth.png"), depth_mm)
        record = {
            "status": "planned",
            "dataset": dataset,
            "run": int(rec.get("run", 0)),
            "recording_id": None,
            "gripper": GRIPPER,
            "product": product,
            "grasp_mode": rec.get("mode", ""),
            "detector_point": {"x": u, "y": v, "u_px": u * W, "v_px": v * H, "source": "gemini"},
            "grasp_world": {"position": g_world.tolist(),
                            "orientation_xyzw": [float(rec["qx"]), float(rec["qy"]), float(rec["qz"]), float(rec["qw"])]},
            "grasp_camera": {"position": g_cam.tolist(), "approach": approach_cam, "finger_axis": fingers_cam},
            "grasp_px": grasp_px,
            "image": {"width": W, "height": H, "frame_id": frame,
                      "stamp": color.header.stamp.sec + color.header.stamp.nanosec * 1e-9,
                      "K": [fx, fy, cx, cy], "distortion": list(info.d), "distortion_model": info.distortion_model,
                      "depth_units": "mm", "depth_aligned_to_color": True},
            "camera_pose_world": {"position": p_wc.tolist(),
                                  "orientation_xyzw": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]},
            "up_camera": up_cam,
            "recorded_at": dt.datetime.now().isoformat(timespec="seconds"),
        }
        with open(os.path.join(sample_dir, "record.json"), "w") as f:
            json.dump(record, f, indent=2)
        self.current = (sample_dir, record)
        gp = "outside the image" if grasp_px is None else f"({grasp_px[0]:.0f}, {grasp_px[1]:.0f}) px"
        self.get_logger().info(f"sample {os.path.basename(sample_dir)}: {product} ({record['grasp_mode']}), "
                               f"Gemini point ({u * W:.0f}, {v * H:.0f}) px, grasp {gp}")

    def _update(self, **fields):
        sample_dir, record = self.current
        record.update(fields)
        with open(os.path.join(sample_dir, "record.json"), "w") as f:
            json.dump(record, f, indent=2)

    def _finish(self, rec, event):
        if self.current is None:
            self.get_logger().warn(f"event={event} for run {rec.get('run')} with no planned sample; ignored")
            return
        if int(rec.get("run", -1)) != self.current[1]["run"] or rec.get("product") != self.current[1]["product"]:
            self.get_logger().warn(f"event={event} (run {rec.get('run')}, {rec.get('product')}) does not match the planned "
                                   f"sample (run {self.current[1]['run']}, {self.current[1]['product']}); ignored")
            return
        rid = rec.get("recording_id")
        self._update(status=event, recording_id=None if rid in (None, "", "none") else rid)
        self.get_logger().info(f"sample {os.path.basename(self.current[0])}: {event}"
                               + (f", recording {rid}" if event == "saved" else ""))
        self.current = None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"sample root (default {DEFAULT_OUT})")
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
