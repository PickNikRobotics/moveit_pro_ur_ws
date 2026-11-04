#!/usr/bin/env python3
"""Report and fit the samples written by the `Check Wrist Camera By Touch-Off` Objective.

Each sample holds, in world: the point the wrist camera pipeline estimated for a clicked pixel,
grasp_link at the snapshot, and grasp_link after the operator touched the cup tip onto the feature.

For every sample it prints the error (touched minus estimated), split into the part along the cup
axis at the touch and the part across it. The across part is what the grasp descent cannot correct.

It then fits one rigid correction X of the camera pose in grasp_link: moving each estimate into the
snapshot grasp_link frame (q) and each touch into the same frame (r), X minimizes |X q - r|. That is
the correction to apply to the camera mount if the error really is the mount. A leave-one-out error
says whether it is: a fit that only works on the samples it was fitted to means the error changes
with the arm pose (forward kinematics, depth bias), not a fixed mount offset.

Usage: scripts/wrist_touch_off_report.py [samples.yaml] [--prefix touch]
See design_docs/wrist_camera_touch_off_check.md.
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import yaml

DEFAULT_FILE = (
    Path(__file__).resolve().parent.parent
    / "src/picknik_009_ur5e_config/objectives/calibration/wrist_touch_off_samples.yaml"
)
PARTS = ("estimated_world", "snapshot_grasp_link_world", "touched_grasp_link_world")


def pose_matrix(entry):
    p = entry["pose"]["position"]
    q = entry["pose"]["orientation"]
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [p["x"], p["y"], p["z"]]
    return m


def load_samples(path, prefix):
    data = yaml.safe_load(Path(path).read_text()) or {}
    pattern = re.compile(rf"^({re.escape(prefix)}_\d+)_(.+)$")
    samples = {}
    for key, entry in data.items():
        match = pattern.match(key)
        if match and match.group(2) in PARTS:
            samples.setdefault(match.group(1), {})[match.group(2)] = entry
    complete = {}
    for name, parts in samples.items():
        missing = [p for p in PARTS if p not in parts]
        if missing:
            print(f"skipping {name}: missing {', '.join(missing)}")
            continue
        frames = {parts[p]["header"]["frame_id"] for p in PARTS}
        if frames != {"world"}:
            print(f"skipping {name}: not all in world ({sorted(frames)})")
            continue
        complete[name] = {p: pose_matrix(parts[p]) for p in PARTS}
    return dict(sorted(complete.items(), key=lambda kv: int(kv[0].rsplit("_", 1)[1])))


def fit_rigid(q, r):
    """Kabsch: R, t minimizing sum |R q_i + t - r_i|^2."""
    qc, rc = q.mean(axis=0), r.mean(axis=0)
    u, _, vt = np.linalg.svd((q - qc).T @ (r - rc))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rot, rc - rot @ qc


def rotation_deg(rot):
    return float(np.degrees(np.arccos(np.clip((np.trace(rot) - 1) / 2, -1.0, 1.0))))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("file", nargs="?", default=str(DEFAULT_FILE))
    parser.add_argument("--prefix", default="touch", help="sample_prefix the Objective ran with")
    args = parser.parse_args()

    samples = load_samples(args.file, args.prefix)
    if not samples:
        sys.exit(f"no complete '{args.prefix}_<n>' samples in {args.file}")

    print(f"{len(samples)} sample(s) from {args.file}\n")
    print(f"{'sample':<12}{'dx':>8}{'dy':>8}{'dz':>8}{'|e|':>8}{'across':>8}{'along':>8}   (mm, world)")
    q_list, r_list, across = [], [], []
    for name, s in samples.items():
        est = s["estimated_world"][:3, 3]
        touched = s["touched_grasp_link_world"][:3, 3]
        err = touched - est
        cup_axis = s["touched_grasp_link_world"][:3, 2]
        along = float(err @ cup_axis)
        lateral = float(np.linalg.norm(err - along * cup_axis))
        across.append(lateral)
        print(
            f"{name:<12}" + "".join(f"{v * 1000:8.1f}" for v in err)
            + f"{np.linalg.norm(err) * 1000:8.1f}{lateral * 1000:8.1f}{along * 1000:8.1f}"
        )
        snap_inv = np.linalg.inv(s["snapshot_grasp_link_world"])
        q_list.append((snap_inv @ np.append(est, 1.0))[:3])
        r_list.append((snap_inv @ np.append(touched, 1.0))[:3])
    print(f"\nacross the cup axis: mean {np.mean(across) * 1000:.1f} mm, max {np.max(across) * 1000:.1f} mm")
    print("('along' includes how far the cup lip sits from grasp_link and how hard it touched.)")

    q, r = np.array(q_list), np.array(r_list)
    if len(q) < 3:
        print("\nAt least 3 samples (6+ from varied poses is better) are needed to fit a correction.")
        return
    rot, t = fit_rigid(q, r)
    before = np.linalg.norm(q - r, axis=1)
    after = np.linalg.norm((q @ rot.T + t) - r, axis=1)
    print("\nFitted camera correction X, in grasp_link (apply as: camera_in_grasp_link_new = X * camera_in_grasp_link):")
    print(f"  translation {np.round(t * 1000, 1)} mm, rotation {rotation_deg(rot):.2f} deg")
    print(f"  residual rms: before {np.sqrt(np.mean(before ** 2)) * 1000:.1f} mm, after {np.sqrt(np.mean(after ** 2)) * 1000:.1f} mm")
    if len(q) >= 4:
        loo = []
        for i in range(len(q)):
            keep = np.arange(len(q)) != i
            rot_i, t_i = fit_rigid(q[keep], r[keep])
            loo.append(np.linalg.norm(rot_i @ q[i] + t_i - r[i]))
        print(f"  leave-one-out rms: {np.sqrt(np.mean(np.square(loo))) * 1000:.1f} mm")
        print("  A leave-one-out error well above the fitted residual means the error is not one fixed mount")
        print("  offset; fix the arm kinematics calibration before trusting X.")


if __name__ == "__main__":
    main()
