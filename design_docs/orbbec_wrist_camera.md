# Orbbec Wrist Camera

## What it does, at a glance

The Orbbec Gemini 305 is the wrist RGB-D camera in the 2F-85 HW config, launched
as `wrist_orbbec` from
[`cameras.launch.xml`](../src/picknik_009_ur5e_hw_config/launch/cameras.launch.xml)
through the `orbbec_camera` driver's `gemini305.launch.py`, under
`/camera/wrist_orbbec/...`. EPick HW still uses the wrist D415, and sim has no
Orbbec. The 2F launch wrapper selects the Orbbec RGB-D preset, disables the wrist
D415, and uses the sole attached D415 as `scene_camera`.

It has two modes, chosen by the `wrist_orbbec_dual_color` launch arg:

- **Dual Color Streams (the default).** Each of the two color lenses on its own
  topic, `left_color/...` and `right_color/...`. The device turns depth off in this
  preset, so there is no `depth/` or point cloud.
- **Default preset.** One `color/image_raw` with both lenses in a single frame,
  side by side, plus `depth/image_raw` and `depth/points`.

A small watchdog,
[`orbbec_stream_watchdog.py`](../src/picknik_009_ur5e_hw_config/scripts/orbbec_stream_watchdog.py),
runs beside the driver and reboots the camera through the driver when its frames stop.

The 2F hardware `Get Wrist Image` / `Get Wrist Point Cloud` subtree variants
read the Orbbec color and registered-cloud topics; EPick hardware still reads the
D415 under `wrist_mounted_camera`. The Gemini's driver publishes eight distortion
coefficients while labeling them `plumb_bob`; the launch relays its color
CameraInfo through a small adapter that labels the eight-coefficient model
`rational_polynomial` without changing the intrinsics or coefficients, as required
by MoveIt Pro 10.2's calibration input validation.

The 2F driver wrapper publishes `d415_mount_link` to `wrist_orbbec_link` from the
`wrist_orbbec_mount` section in `cell_calibration.yaml`. That section begins as a
temporary estimate based on the former D415 body pose. The 2F-specific [shelf-board
calibration](orbbec_shelf_board_calibration.md) now uses MoveIt Pro 10.2's eye-in-hand
solver, converts its optical-frame result to the Orbbec root mount, and writes a
candidate for explicit review and application. The 2F `Get Wrist Image` and
`Get Wrist Point Cloud` switching subtrees route to the Orbbec color and
color-registered point-cloud streams. The latter depends on the Gemini 305 driver's
color registration setting; verify the live point-cloud dimensions and frame against
the color image before relying on pixel-to-3D projection.

## Why this shape

- **Driver from apt, not a submodule.** `orbbec_camera` is released into the ROS
  Jazzy apt repo, and the hw config's `package.xml` `exec_depend` on it lets the
  `Dockerfile`'s `rosdep install` put it in the image. We carry no patch, so
  vendoring it would only add a pointer to bump. This matches the
  `moveit_pro_orbbec` plan in
  [`hardware_modularity.md`](../src/shelf_stocking/design_docs/hardware_modularity.md).
- **The Orbbec is a 2F-only camera variant.** The established `wrist_mounted_camera`
  contract remains intact for sim and EPick HW. The 2F leaf points its existing
  wrist subtree IDs at the Orbbec topics and attaches the driver's
  `wrist_orbbec_link` below the adapter mount. The old D415 robot-description links
  remain for SRDF and collision compatibility, but no wrist D415 driver is launched.
- **The Orbbec mount remains separate from the D415 mount.** `wrist_orbbec_mount`
  is consumed by the 2F driver launch; `wrist_camera_mount` remains owned by the
  D415 xacro and EPick calibration Objectives. Applying a reviewed Orbbec candidate
  writes a plain pose section, after which launch accepts the missing temporary
  marker as calibrated and still checks that the parent is `d415_mount_link`.
- **The scene D415 selection is currently unpinned in the 2F wrapper.** That is
  unambiguous while exactly one D415 is connected. If another RealSense device is
  attached, pin the scene camera with its SDK serial (not the USB serial printed
  by `lsusb`).
- **Default RGB-D for the 2F config, dual color remains opt-in for other runs.**
  The Orbbec include uses launch arguments rather than the driver's shipped
  `gemini305_dual_color.yaml`, because that YAML wins during parameter merging and
  carries `camera_name: camera` and an empty serial, which would rename frames and
  drop the serial pin. The dual-color preset has no depth, so it is not suitable
  for 2F perception.
- **A watchdog that reboots through the driver, not a relaunch or a USB reset.**
  See the stall note below: the driver's `reboot_device` service is the only
  recovery found that works in place, as the workspace user, without root.
- **The `/camera` prefix is pushed in the launch file.** The upstream launch uses
  `camera_name` as both the node name and the namespace, so it cannot take
  `camera/wrist_orbbec`. A `push_ros_namespace` around the include adds it.

## Notes and gotchas

- **About one start in two or three, the camera opens and then sends nothing.**
  The driver logs every stream as enabled and reports no error. `device_status`
  shows frame rates of zero, and the UI shows nothing or a single frame. It
  happens with or without a USB reset before the start, so it is not state left by
  the previous run. Relaunching the node only sometimes clears it. What does clear
  it every time is the driver's `reboot_device` service, which the watchdog calls
  when the watched `camera_info` goes quiet. A start that needed it streams within
  about a minute instead of a few seconds, and the watchdog logs the reboot. As a
  manual alternative, re-enumerating the camera through its sysfs `authorized` file
  also works, but needs root. Check `product` / `serial` in that directory first,
  because the ZED sits on the same bus. The likely cause is the kernel `uvcvideo`
  driver re-claiming the camera after every reset or node exit, while the SDK's
  libuvc backend has to take each interface back from it. That is a hypothesis.
  The V4L2 backend, which would avoid it, cannot find the camera inside the
  container ("No v4l device found for port ..."), even with its `/dev/video*`
  nodes present.
- **The Orbbec include pins its own `device_type` (and a few others), and that is
  load-bearing.** In ROS 2 launch, `<include>` args and an included file's declared
  defaults stay set for the siblings after it, and `gemini305.launch.py` builds the
  node's parameters by reading every argument it declares from the launch context.
  So without the pin it inherits the D415 blocks' `device_type=D415`. The node then
  connects, logs `ob_camera_node_ is nullptr` and `Initialize device cost 0 ms`, and
  the container dies with exit code 11 (segfault). A hand launch of the same include
  works, which hides the cause. Before adding a camera block above the Orbbec one,
  check its arg names against the ones `gemini305.launch.py` declares. The
  RealSense wrapper's "Parameter 'launch_wrist_orbbec' is not supported" warning is
  the same leak in the other direction, and harmless.
- **Host udev rule required.** The Orbbec SDK opens the camera through
  `/dev/bus/usb` (libusb). Without the vendor rule, that node is `root:root 0664`,
  and the drivers container runs its nodes as the workspace user, so the node
  starts, logs `Device with serial number ... not found` and retries forever,
  much like a missing RealSense serial. A `docker exec` as root works anyway, which
  makes a hand test misleading. **The rule the driver ships
  (`share/orbbec_camera/udev/99-obsensor-libusb.rules`, as of 2.7.6) does not list
  the Gemini 305's product ID (`2bc5:0840`)**, so installing that rule as it is
  changes nothing for this camera. Add a rule for `0840` modelled on its Gemini 330
  lines, in its own file in `/etc/udev/rules.d/`, then reload udev and trigger the
  `2bc5` USB devices, and check `ls -l /dev/bus/usb/<bus>/<dev>` shows `0666`.
  `/dev/bus/usb` is bind-mounted live into the drivers container, so the new mode
  shows up there at once, but a node that started without access keeps logging
  "not found" even though it retries. Restart the stack, or stop only that node's
  `component_container` with SIGINT (the drivers launch does not shut down when it
  exits) and run the `cameras.launch.xml` Orbbec block again as the workspace user.
- **In the Default preset the two lenses do not split at the middle of the
  frame.** The seam is at a fixed column well right of centre, and the two sides
  show very different views. So cropping the combined image in half is not a
  substitute for the Dual Color Streams preset.
- **`Failed to get right_color extrinsic` in dual color mode is harmless for the
  images.** Both lenses still stream. It only means the driver publishes no
  left-to-right transform.
- **A stale `/dev/shm/orbbec_device_lock` left by a different user** makes the
  next start fail with `Failed to open shared memory`. Delete it. This only happens
  after a hand launch as root.
- **The startup root mount is not trusted until calibrated.** Before applying a
  reviewed solve, the launch publishes the legacy D415-derived estimate. After
  application, restart and verify the Orbbec cloud against the shelf and board in
  the MoveIt Pro 3D Visualizer before relying on wrist-derived 3D poses for grasping.
- **The colored cloud topic and pixel grid are Orbbec-specific.** In the Default
  preset, registration to COLOR is requested and the 2F subtree reads the driver's
  registered cloud output. Confirm on the installed driver that image, camera
  info, cloud organization, and header frame match the projection behavior expected
  by `GetPoseFromPixelCoords` and `GetMasks3DFromMasks2D`.
- **It shares a USB 3 root hub with the ZED** on the current Agent PC (they are
  the only two devices on that bus). If either starts dropping frames, try moving
  one of them to another controller.
