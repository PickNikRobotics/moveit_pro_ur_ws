# Robot Stand Corner Pole

## At a glance

The 2F-85 hardware planning scene models the added 80/20 stand pole as a collision box in [`Load Calibrated Scene`](../src/picknik_009_ur5e_2f85_hw_config/objectives/load_calibrated_scene.xml). Its position was measured using live Robotiq fingertip touch-offs, accounting for fingertip geometry beyond TF. The old thin camera-mount plate over the drop area is removed from the scene; the physical plate mounted on the stand pole remains. These objects are specific to this hardware config, not the shared robot description or EPick/simulation scenes. The 2F-85 scene also removes the old table-side shelf-camera post, crossbar and clamp collision objects, since that camera is no longer present.

## Why this shape

- **Planning-scene primitives are enough.** The pole and its flat camera plate are fixed rectangular extrusions, so world-frame collision boxes are simpler than adding URDF geometry and do not alter the common robot model.
- **Keep the hardware-specific change in the 2F-85 leaf.** The pole belongs to this physical setup, not to the inherited EPick or MuJoCo configuration.

## Notes and gotchas

- The touch-off had the fingertips against the pole's front face, with the gripper pointing approximately along world −Y. `grasp_link` is between the fingers and is not the contact point. The fingertip-TF estimate still left the collision box intersecting the real fingertips, so the pole was shifted 3.5 cm away from the robot (−world Y), matching the measured finger reach.  Vertically it is anchored to the stand mesh's bottom, used as the floor reference. The pose is world-fixed, not attached to TF; moving the pole or stand requires another fingertip touch-off and an Objective update.
- Both object IDs are stable and each box is overwritten when `Load Calibrated Scene` runs again. The plate is mounted on the pole's +Y face, with its 7-inch axis projecting along world +X, perpendicular to that face. Its −X edge aligns with the pole's −X edge so the plate extends outward in +X; its height comes from the fingertip touch-off against the top edge. The pole's 9-inch +X shift from its original touch-off position is independent of this plate alignment. At scene cleanup, the 2F-85 config also removes the obsolete `vla_cam_rig_post`, `vla_cam_rig_crossbar` and `vla_cam_rig_clamp` IDs so objects from a previously loaded shelf-camera scene do not remain.
- This collision object is planning-scene geometry; it is not added to the MuJoCo scene or the UR stand mesh.
