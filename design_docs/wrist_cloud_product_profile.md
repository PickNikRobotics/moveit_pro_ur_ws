# Wrist Cloud Product Profile

## What it does, at a glance

[`scripts/measure_product_profile.py`](../scripts/measure_product_profile.py) measures a shelf product's width and depth at every height from the 2F-85 wrist Orbbec cloud. It is a diagnostic for one question: can the wrist cloud size a 2F-85 front grasp from the product itself, instead of from its `exemplars.yaml` catalog size? The question comes from the mustard. Its catalog size is its narrowest part, the cap, so the planned grasp is sized for the cap and not for the body the fingers close on.

It has two steps:
- **`snap`** runs in the runtime container with the stack up. It saves the colour image, `camera_info` and a few clouds, each with its world transform at its own stamp.
- **`measure`** runs offline. It keeps the cloud points that project into a pixel box drawn around the product, drops the board and whatever stands behind, and reports per horizontal slice:
  - the trimmed width along the rack;
  - the visible depth, from the front face to the outline;
  - a depth fitted from the curvature of the front.

At the pipeline's front grasp height, `measure` also prints where `Compute 2F-85 Shelf Grasps` puts `grasp_link` and the finger tips from the catalog size, and where it would put them from the measured one. It writes a points overlay, a width-and-depth-against-height plot and a CSV next to the snapshot. The script's docstring has the commands.

## Why this shape

- **The product is picked by a pixel box, not by Gemini and SAM3.** The point is to test the cloud alone. A mask from the pipeline would mix the mask's errors into the measurement. The box comes from the raw colour image, and points are projected into it with the camera's distortion. The cloud is colour-registered and unorganized, so there is no pixel index to use.
- **Snapshots are saved, measuring is offline.** One snapshot can be re-measured with a different box, board height, depth gate or depth scale. Comparing with calipers takes several tries.
- **The width is the same trimmed extent the pipeline takes.** `ComputeProductKeepoutBox` trims points from each end. `measure` uses the grasp XML's trim fraction by default, so its width is the one a cloud-sized grasp would see.
- **Depth is reported two ways, because a front view does not see the back.** For a round or oval section, the visible depth is about half the body depth, but the outline's points are the noisiest in the cloud. The fitted depth uses only the middle of the front: the front profile of an oval seen square-on is linear in its two unknowns once the width is known. The two should agree on a clean bottle. If they don't, the outline is not good enough to size a grasp from.
- **The grasp numbers are read from the source, not copied.** The port defaults come from `compute_2f85_shelf_grasps.xml`, and the catalog size and `grasp_height_fraction` from `exemplars.yaml`. So the comparison follows any retuning.

## Notes and gotchas

- **When the back-face cap binds, the bottle is held on the finger tips only.** That cap keeps the tips short of the product's back face. The grasp table shows `grasp_link` against the front face and the tips against both faces. When the cap binds, `grasp_link` can sit in front of the front face and the pads' middle is in front of the product. Only the stretch from the front face to the tips holds it. That is the shallow mustard grasp.
- **The cloud stops at the camera's minimum range.** From close in (a pre-grasp, or the arm parked over a product), the near products and their lids have no depth at all. Only the board around them survives, and the measurement is of the board. `measure` prints the share of the box that has depth and the nearest point in the whole cloud, and warns when coverage is low. Snap from the pick's start view, which is what the grasp is planned from.
- **Mustards stand back to back.** The bottle behind starts about one body depth back, so the depth gate has to stay under the product's depth. The overlay shows which points were kept; check it before trusting a row.
- **The board is found, not known.** By default its height is the densest band at the bottom of the box. At a grazing view the board's points spread over a band, and the board in front of the base is nearer than the product's face, so a wrong board height sets a wrong front face. Pass the calibrated rack height (`--board-z`) when the printed value looks off.
- **The gripper's own fingers are in view.** They are nearer than the product, so a minimum range drops them before the box is applied.
- **`--depth-scale` tests the short-depth hypothesis.** The Orbbec reads depth short (see [`orbbec_wrist_camera.md`](orbbec_wrist_camera.md)). Scaling the cloud's range shows how much that moves the front face. Width scales with range too, but at product size that is under a millimetre.
- **`snap` needs the ROS environment, so it cannot run under `python3 -I`.** `-I` drops the `PYTHONPATH` that `/opt/ros/jazzy/setup.bash` sets. It also needs the container's CycloneDDS file exported (`CYCLONEDDS_URI`); without it, node creation fails with "Failed to find a free participant index". `measure` needs only numpy, cv2 and PyYAML and runs under `-I`.
