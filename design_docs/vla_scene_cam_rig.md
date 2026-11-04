# VLA Scene Cam Rig

## What it does, at a glance

A cross-shaped 80/20 camera rig (a vertical post with a crossbar bolted to its far side, off-center along Y) stands on the table on the world −X side of the bin (the operator's right). It is modeled as world-frame collision boxes, one per bar, added by [`Load Calibrated Scene`](../src/picknik_009_ur5e_config/objectives/load_calibrated_scene.xml) next to the table and the camera mount plate. The object ids start with `vla_cam_rig_`. The rig is still being modeled, so more pieces will be added.

## Why this shape

- **Touch-off, not AprilTag calibration.** The rig has no tag. Its pose came from touching the suction cup to the bar intersection and reading `grasp_link` from TF. That puts it in the same category as the table and camera mount plate: fixed workcell assumptions hard-coded in the XML, not values loaded from `objectives/calibration/*.yaml`. Re-running the calibration does **not** move it.
- **Primitive boxes, not a URDF.** Each bar is a single box. A URDF (like `workcell.urdf`) only pays off once there are enough parts that sharing one pose becomes useful. If the rig grows a lot, consider moving it into a `description/` URDF placed with one `AddURDF`.

## Notes and gotchas

- **Re-measure if the rig moves.** Nothing tracks it. The touch-off reading and the arithmetic that turns it into bar centers are written in the XML comment above the boxes. Update both together.
- **Contact face vs. centerline.** A touch-off records a bar's +X *face*, not its centerline.
- **The bars don't intersect.** The post is on the bin side (+X). The crossbar, which carries the camera, is bolted to the post's outside (−X) face, so the post's centerline is one full bar width closer to the bin than the crossbar's. If you model the bars as intersecting, or put the crossbar on the wrong side, one of them ends up about 4 cm off. The crossbar is also off-center: most of its length points away from the shelf.
- **One touch-off anchors everything.** The whole rig comes from a single reading: the cup touching the post's bin-side face at the cross. The crossbar and clamp are placed relative to the post using the measured bar dimensions, not separate readings. To re-measure, repeat that one touch-off and re-derive the rest using the arithmetic in the XML comment.
- **Eyeballed gaps are not reliable.** An earlier "about 2 inches away" estimate, read off a teleop pose, put the crossbar about 1.7 cm further out than the direct contact did. Use contact, not estimated stand-off.
- **The camera clamp is a rough bounding box.** It is one box, flush with the crossbar's shelf-side end. That is deliberate: it's conservative keep-out, not geometry to plan close against. In X/Z it is aligned to the crossbar's top, robot-side (+X) corner, so it hangs below the bar and extends away from the robot instead of overhanging toward it.
- **Planning scene only.** The MuJoCo sim scene (`picknik_009_ur5e_config_sim/mjcf/assets/additions.xml`) does not include the rig. Sim physics won't collide with it, but sim planning will still avoid it, because `Load Calibrated Scene` is shared by both configs.
