#!/usr/bin/env python3
"""Check that every free product stays inside the sim tote from each start state.

Usage: check_sim_products_in_bin.py <path to mjcf/scene.xml> [seconds]

Runs the model's default state and every keyframe for a few seconds and lists the
products that end outside the calibrated_bin walls. The products are laid out in world,
so run this after a calibration moves the tote. Needs the mujoco Python package.
"""

import os
import sys

import mujoco
import numpy as np

# Slack inside the tote walls before a product counts as outside.
WALL_MARGIN = 0.01


def inner_half_extents(model):
    body = model.body("calibrated_bin").id
    x = y = None
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body:
            continue
        name = model.geom(g).name
        if name.endswith("_right"):
            x = model.geom_pos[g][0] - model.geom_size[g][0]
        if name.endswith("_front"):
            y = model.geom_pos[g][1] - model.geom_size[g][1]
    return x - WALL_MARGIN, y - WALL_MARGIN


def outside(model, data, joints, bin_pos, half_x, half_y):
    names = []
    for j in joints:
        p = data.joint(j).qpos[:3] - bin_pos
        if abs(p[0]) > half_x or abs(p[1]) > half_y or p[2] < -0.1:
            names.append(j)
    return names


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    seconds = float(sys.argv[2]) if len(sys.argv) == 3 else 2.0
    # MuJoCo misplaces mesh files when given a relative path with a folder in it.
    model = mujoco.MjModel.from_xml_path(os.path.abspath(sys.argv[1]))
    data = mujoco.MjData(model)
    joints = [model.joint(j).name for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    half_x, half_y = inner_half_extents(model)
    failed = False
    for key in [-1] + list(range(model.nkey)):
        mujoco.mj_resetData(model, data)
        if key >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key)
        mujoco.mj_forward(model, data)
        bin_pos = data.body("calibrated_bin").xpos.copy()
        for _ in range(int(seconds / model.opt.timestep)):
            mujoco.mj_step(model, data)
        name = model.key(key).name if key >= 0 else "(model default)"
        out = outside(model, data, joints, bin_pos, half_x, half_y)
        failed |= bool(out)
        print(f"{name}: {'OK' if not out else 'outside the tote: ' + ', '.join(out)}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
