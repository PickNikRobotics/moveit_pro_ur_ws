# Calibrate Bin

## At a glance

[`Calibrate Bin`](../src/picknik_009_ur5e_2f85_hw_config/objectives/calibrate_bin.xml) is the 2F-85 hardware calibration for the open drop bin on the table. The operator closes the gripper, teleoperates the left fingertip to the bin's front-left outside rim corner (robot-facing edge), and continues to save the measured fingertip pose in `cell_calibration.yaml`. The 2F-85 [`Load Calibrated Scene`](../src/picknik_009_ur5e_2f85_hw_config/objectives/load_calibrated_scene.xml) reads that corner, derives a world-aligned bin pose, and adds four wall boxes; the table is the floor. Its `Calibrated Bag` compatibility subtree supplies the interior dimensions to the existing 2F-85 drop logic.

## Why this shape

- **Measure a physical reference, derive the rest from the known bin geometry.** Only one touched corner is needed because the bin is treated as square to world and its size is fixed. The saved pose is the gripper fingertip frame; the compatibility subtree offsets from that frame to the mesh contact point, removes the fingertip orientation, then derives the bin centre and floor height.
- **Keep the shared drop interface while replacing the bag.** `Gentle Bag` and its callers already exchange the drop container as `Calibrated Bag`'s `bag_*` values. The 2F-85 config retains that interface but sources its pose from this calibration and its dimensions from the rigid bin, so the generic pick-and-drop subtrees do not need a gripper-specific rewrite.
- **The planning scene models only what the planner needs.** Four thin wall boxes leave the top open and omit a floor because the bin sits on the modeled table. The walls are rigid obstacles, so the old paper-bag collision allowance for forearm and wrist contact is removed.
- **Joint jog needs a UI mode selection before controller switching.** `Request Teleoperation` leaves the mode at waypoint when starting, then runs the joint-jog branch only after the UI selects joint jog. That branch activates the controller list from the selected UI mode. `Calibrate Bin` starts at waypoint mode intentionally; select joint jog in the UI before jogging the finger, or controller switching fails on an unset `{controllers}` key.

## Notes and gotchas

- The calibration section is now `calibrated_bin_front_left_corner`; it is separate from any earlier back-corner measurement. Calibrate again before loading the bin so the scene uses the new front-corner reference.
- Run `Calibrate Bin` after moving or replacing the bin, before any Objective that loads the 2F-85 scene. It automatically runs `Load Calibrated Scene` after saving so the new pose can be inspected in the MoveIt Pro 3D Visualizer.
- The touched corner is the front-left corner from the robot's viewpoint (world +X, +Y). Keep the bin axes aligned with world X/Y; the saved fingertip orientation is deliberately not used to rotate the bin.
- The calibration stores the fingertip-link origin, not the contact vertex. The local contact offset is tied to the left fingertip collision mesh and the gripper must be closed for this touch-off. If the gripper description, finger state or contact feature changes, re-check that offset before trusting the bin placement.
- The physical outer dimensions include the wall thickness; `Calibrated Bag` exposes the corresponding interior dimensions to product placement. The dimensions live in that XML and the wall geometry/thickness lives in `Load Calibrated Scene`.
- The YAML section is shared with the scene loader and is written with `SavePoseStampedToYaml` using a named namespace, so rerunning calibration updates only this pose record while retaining the rest of `cell_calibration.yaml`.
- This calibration is hardware-only. The inherited MuJoCo scene still generates its tote and cameras from the EPick board calibration inputs; it does not use this 2F-85 bin pose.
