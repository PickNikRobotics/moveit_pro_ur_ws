#!/usr/bin/env python3
"""Fill the calibrated-scene MJCF template from the calibration files.

Usage: generate_calibrated_scene_mjcf.py <cell_calibration.yaml> <board_offsets.yaml> <template> <output>

Repeats the placement in Place Shelving From Wall Board and Place Tote From Tote Board
(picknik_009_ur5e_config/objectives) so the MuJoCo bodies sit where the planning scene puts
them. Keep the two in step. See design_docs/board_based_calibration.md.
"""

import string
import sys

import yaml

IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)
# The racks' half-turn about world z, as TransformPose applies it in the placement subtree.
HALF_TURN_XYZW = (0.0, 0.0, 1.0, 0.0)


def rotate(q_xyzw, v):
    """Rotate vector v by the unit quaternion q (x, y, z, w)."""
    x, y, z, w = q_xyzw
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    # v' = v + 2w(q x v) + 2 q x (q x v)
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return [v[0] + 2 * (w * cx + ccx), v[1] + 2 * (w * cy + ccy), v[2] + 2 * (w * cz + ccz)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def section_pose(calibration, name):
    try:
        pose = calibration[name]["pose"]
        frame = calibration[name]["header"]["frame_id"]
    except (KeyError, TypeError) as error:
        sys.exit(f"cell_calibration.yaml: section '{name}' is missing or incomplete ({error})")
    if frame != "world":
        sys.exit(f"cell_calibration.yaml: section '{name}' is in frame '{frame}', expected 'world'")
    position = [float(pose["position"][k]) for k in "xyz"]
    orientation = tuple(float(pose["orientation"][k]) for k in "xyzw")
    return position, orientation


def offset(offsets, name):
    try:
        values = [float(v) for v in str(offsets[name]["xyz"]).split(";")]
    except (KeyError, TypeError, ValueError) as error:
        sys.exit(f"board_offsets.yaml: section '{name}' is missing or malformed ({error})")
    if len(values) != 3:
        sys.exit(f"board_offsets.yaml: section '{name}' needs three values 'x;y;z'")
    return values


def mjcf_pos(p):
    return " ".join(f"{v:.8f}" for v in p)


def mjcf_quat(q_xyzw):
    x, y, z, w = q_xyzw
    return " ".join(f"{v:.8f}" for v in (w, x, y, z))


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    calibration_path, offsets_path, template_path, output_path = sys.argv[1:]
    with open(calibration_path) as f:
        calibration = yaml.safe_load(f)
    with open(offsets_path) as f:
        offsets = yaml.safe_load(f)

    poses = {}
    # Shelving: board corner offset in the board frame, then world-aligned offsets.
    board_p, board_q = section_pose(calibration, "wall_board")
    corner = add(board_p, rotate(board_q, offset(offsets, "wall_corner_in_wall_board")))
    poses["shelf_back"] = (add(corner, offset(offsets, "wall_center_from_wall_corner")), IDENTITY_XYZW)
    poses["top_shelf"] = (add(corner, offset(offsets, "top_rack_from_wall_corner")), HALF_TURN_XYZW)
    poses["bottom_shelf"] = (add(corner, offset(offsets, "bottom_rack_from_wall_corner")), HALF_TURN_XYZW)
    poses["shelf_base"] = (add(corner, offset(offsets, "shelf_base_from_wall_corner")), IDENTITY_XYZW)
    # Tote: the same two-step placement from the tote board.
    board_p, board_q = section_pose(calibration, "tote_board")
    corner = add(board_p, rotate(board_q, offset(offsets, "tote_outer_corner_in_tote_board")))
    poses["bin"] = (add(corner, offset(offsets, "tote_origin_from_outer_corner")), IDENTITY_XYZW)
    # Fixed cameras: the stored root-link poses as they are.
    for name in ("scene_camera_link", "shelf_cam_camera_link"):
        poses[name] = section_pose(calibration, name)

    values = {}
    for name, (p, q) in poses.items():
        values[f"{name}_pos"] = mjcf_pos(p)
        values[f"{name}_quat"] = mjcf_quat(q)
    with open(template_path) as f:
        template = string.Template(f.read())
    try:
        text = template.substitute(values)
    except (KeyError, ValueError) as error:
        sys.exit(f"{template_path}: unknown or malformed placeholder ({error})")
    text = text.replace("    TEMPLATE. The sim package build fills", "    GENERATED, do not edit. The sim package build filled", 1)
    with open(output_path, "w") as f:
        f.write(text)


if __name__ == "__main__":
    main()
