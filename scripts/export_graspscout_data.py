#!/usr/bin/env python3
"""Export the GraspScout samples whose Trainer episode was kept, for fine-tuning GraspScout.

graspscout_data_recorder.py writes one sample per pick the Unshelve Product Loop plans. A sample
counts as ground truth only if its pick was saved AND its Trainer episode still exists: deleting a
bad demo in the Trainer drops its sample here too. Gemini's point says which product is where; the
grasp that then worked (and was kept) says how it can be picked.

For every kept sample this writes, under --out:
  crops/<label>/<sample>.png   the product's SAM3-masked crop, labelled by Gemini: new gallery views
                               for GraspScout's matcher (copied into --add-to-gallery DIR if given,
                               the same layout enroll_cli writes)
  masks/<sample>.png           that product's mask (255 inside)
  grasps.jsonl                 one line per sample: image, depth and mask paths, K, the gravity-up
                               vector in the camera frame, the label, Gemini's point, the executed
                               grasp (pixel, 3D in the camera frame, approach and finger axes, front
                               or top), the gripper, and the Trainer dataset and recording it came from
With --include-discarded, failed picks are added to grasps.jsonl too, marked "outcome": "failed"
(negatives); they never become gallery crops.

Run with GraspScout's own Python (it uses GraspScout's SAM3 and crop code):
  ~/GraspScout/.venv/bin/python -I ~/moveit_pro_ur_ws/scripts/export_graspscout_data.py \
      --graspscout ~/GraspScout --gallery ~/new_GraspScout/GraspScout/gallery/mine --out ~/graspscout_finetune
"""

import argparse
import glob
import json
import os
import shutil
import sys

import numpy as np

DEFAULT_DATA = "~/.local/share/moveit_pro/graspscout_data"
DEFAULT_RECORDINGS = "~/.local/share/moveit_pro/trainer/recordings"


def episode_kept(recordings, dataset, recording_id):
    """The Trainer keeps a recording as recordings/<dataset>/<id>/ with non-empty .mcap files; a
    discarded or deleted one is gone or empty."""
    if not recording_id:
        return False, "no recording id (recorded without a Trainer episode)"
    d = os.path.join(recordings, dataset, recording_id)
    if not os.path.isdir(d):
        return False, "episode deleted"
    if not any(os.path.getsize(f) > 0 for f in glob.glob(os.path.join(d, "*.mcap"))):
        return False, "episode empty (discarded)"
    return True, "kept"


def product_mask(gs, rgb, u_px, v_px):
    """SAM3 masks the way GraspScout.detect makes them; the best-scoring one that contains the point."""
    import cv2
    H, W = rgb.shape[:2]
    feats = gs._encode(rgb)
    best = None
    for tf, tm in gs.prompts:
        pm, sc = gs._decode(feats, tf, tm)
        for j in np.where(sc >= gs.cfg.second_lo)[0]:
            m = cv2.resize(pm[j], (W, H), interpolation=cv2.INTER_LINEAR) >= 0
            ui, vi = int(round(u_px)), int(round(v_px))
            if m.sum() > 50 and 0 <= vi < H and 0 <= ui < W and m[vi, ui]:
                if best is None or sc[j] > best[0]:
                    best = (float(sc[j]), m)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--data", default=DEFAULT_DATA, help=f"recorder sample root (default {DEFAULT_DATA})")
    ap.add_argument("--recordings", default=DEFAULT_RECORDINGS, help="Trainer recordings root")
    ap.add_argument("--graspscout", required=True, help="GraspScout folder (its graspscout package and models/)")
    ap.add_argument("--gallery", required=True, help="GraspScout gallery the service runs with (it needs one to start)")
    ap.add_argument("--out", required=True, help="export folder")
    ap.add_argument("--dataset", default=None, help="only this Trainer dataset")
    ap.add_argument("--include-discarded", action="store_true", help="add failed picks to grasps.jsonl as negatives")
    ap.add_argument("--add-to-gallery", default=None, help="also copy the crops into this gallery (one folder per label)")
    ap.add_argument("--device", default="auto")
    a = ap.parse_args()

    sys.path.insert(0, os.path.expanduser(a.graspscout))
    from PIL import Image
    from graspscout.core import Config, GraspScout, crop

    data, recordings, out = (os.path.expanduser(p) for p in (a.data, a.recordings, a.out))
    gs = GraspScout(Config(models=os.path.join(os.path.expanduser(a.graspscout), "models"),
                           gallery=os.path.expanduser(a.gallery), device=a.device))
    for sub in ("crops", "masks"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    counts = {}
    with open(os.path.join(out, "grasps.jsonl"), "w") as manifest:
        for rec_path in sorted(glob.glob(os.path.join(data, "*", "*", "record.json"))):
            sample_dir = os.path.dirname(rec_path)
            name = os.path.basename(sample_dir)
            rec = json.load(open(rec_path))
            if a.dataset and rec.get("dataset") != a.dataset:
                continue
            if rec["status"] == "saved":
                kept, why = episode_kept(recordings, rec["dataset"], rec.get("recording_id"))
                outcome = "success" if kept else None
            elif rec["status"] == "discarded" and a.include_discarded:
                kept, why, outcome = False, "discarded pick", "failed"
            else:
                kept, why, outcome = False, rec["status"], None
            if outcome is None:
                counts[why] = counts.get(why, 0) + 1
                continue
            rgb = np.asarray(Image.open(os.path.join(sample_dir, "rgb.png")).convert("RGB"))
            dp = rec["detector_point"]
            found = product_mask(gs, rgb, dp["u_px"], dp["v_px"])
            mask_rel = crop_rel = None
            if found is not None:
                score, m = found
                mask_rel = os.path.join("masks", name + ".png")
                Image.fromarray((m * 255).astype(np.uint8)).save(os.path.join(out, mask_rel))
                if outcome == "success" and rec["product"] != "unknown":
                    crop_rel = os.path.join("crops", rec["product"], name + ".png")
                    os.makedirs(os.path.dirname(os.path.join(out, crop_rel)), exist_ok=True)
                    crop(rgb, m).save(os.path.join(out, crop_rel))
                    if a.add_to_gallery:
                        dst = os.path.join(os.path.expanduser(a.add_to_gallery), rec["product"])
                        os.makedirs(dst, exist_ok=True)
                        shutil.copy(os.path.join(out, crop_rel), os.path.join(dst, "unshelve_" + name + ".png"))
            line = {
                "sample": name,
                "outcome": outcome,
                "label": rec["product"],
                "gripper": rec["gripper"],
                "grasp_mode": rec["grasp_mode"],
                "rgb": os.path.join(sample_dir, "rgb.png"),
                "depth_mm": os.path.join(sample_dir, "depth.png"),
                "mask": os.path.join(out, mask_rel) if mask_rel else None,
                "crop": os.path.join(out, crop_rel) if crop_rel else None,
                "width": rec["image"]["width"], "height": rec["image"]["height"],
                "K": rec["image"]["K"],
                "up_camera": rec["up_camera"],
                "detector_point": dp,
                "grasp_px": rec["grasp_px"],
                "grasp_camera": rec["grasp_camera"],
                "dataset": rec["dataset"],
                "recording_id": rec.get("recording_id"),
            }
            manifest.write(json.dumps(line) + "\n")
            key = f"{outcome}{'' if found else ' (no SAM3 mask at the point)'}"
            counts[key] = counts.get(key, 0) + 1
    print("samples:", ", ".join(f"{v} {k}" for k, v in sorted(counts.items())) or "none")
    print(f"wrote {os.path.join(out, 'grasps.jsonl')}")
    labels = sorted(os.listdir(os.path.join(out, "crops")))
    for lab in labels:
        print(f"  crops/{lab}: {len(os.listdir(os.path.join(out, 'crops', lab)))}")
    if a.add_to_gallery:
        print(f"copied the crops into {a.add_to_gallery}; restart the GraspScout service (or use its /enroll page) "
              "so it loads the new views")


if __name__ == "__main__":
    main()
