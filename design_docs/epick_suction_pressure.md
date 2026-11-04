# EPick Suction Pressure

## At a glance

The Robotiq EPick driver's "how hard to pull vacuum" target is set in the URDF as `grip_max_vacuum_pressure` (kPa, negative = below atmospheric). The upstream `epick_single_suction_cup` macro from the `ros2_epick_gripper` submodule does **not** forward this parameter to the inner `epick_driver_ros2_control` call, so this workspace inlines the suction-cup macro body directly into [`picknik_009_ur5e.urdf.xacro`](../src/picknik_009_ur5e_config/description/picknik_009_ur5e.urdf.xacro) and passes a workspace-side `xacro:arg` (`epick_grip_max_vacuum_pressure`) through to the driver.

The arg lives next to the other `epick_*` args near the top of the same file; the inlined macro body lives at the original `epick_single_suction_cup` call site.

The same inlined call is also where the driver's **serial port** is set, via a second workspace-side arg (`epick_usb_port`). On UR5e 009 the gripper is wired to an FTDI "USB TO RS-485" bridge plugged into the Agent PC, which enumerates as `/dev/ttyUSB0` — the same value as the upstream default, but declared explicitly so a `ros2_epick_gripper` pointer bump cannot move the device out from under the robot. The `Dockerfile` puts the container user in `dialout` for exactly this device, and the `drivers` Compose service is privileged, so no extra device plumbing is needed.

**Do not bind this to `tool_device_name`.** That arg is the socat pty (`/tmp/ttyUR`) that `ur_tool_comms.launch.xml` creates for the arm's *tool-port* RS-485, and it is a different physical path that this robot does not use. Pointing the driver there is a bring-up-stopping mistake rather than a degraded mode: `socat` runs with `waitslave`, so it only dials the robot's tool-comm TCP port at the instant the EPick opens the pty. If the robot is not serving that port, the connect is refused, the EPick's first Modbus read returns nothing, and hardware activation raises — which `ros2_control_node` treats as fatal (`Failed to set the initial state of the component ... to active`), aborting the process and taking the whole `drivers` container down with a launch-level "critical process died". The gripper does not fail soft; the arm never comes up.

## Why this shape

Three approaches were on the table:

1. **Fork the submodule** (`ros2_epick_gripper`, PickNik branch `epick-body-box-collision`) to forward the param through `epick_single_suction_cup`. Cleanest, but requires a submodule commit + pointer bump in this repo for every tweak, and it spends churn on a wrapper-macro signature change that only this workspace needs.
2. **Bump the submodule default** for `grip_max_vacuum_pressure` directly. One-line change, but invisibly affects every downstream consumer of the submodule, and gives this workspace no per-arm knob.
3. **Inline the macro body here** and pass the override on the inner `epick_driver_ros2_control` call. A modest amount of XML duplication, no submodule changes, and the kPa target becomes a normal `xacro:arg` that can be overridden from a launch file.

We picked (3) because the pressure target is the kind of thing we'll want to retune per task / per shelf, and the cost of a submodule round-trip every time outweighs the cost of carrying a duplicated macro body.

## Notes and gotchas

- **AdvancedMode only.** The driver only sends a programmable grip target in `AdvancedMode`. In `AutomaticMode` the value is silently ignored — the gripper just runs at its built-in max. The driver's macro default is `AdvancedMode`, and nothing in this workspace overrides it, so the override is live.
- **Physical ceiling near the unit's spec.** The pump can't pull harder than its rated vacuum; targets near the absolute-vacuum end of the driver's clamp range saturate against that hardware limit.
- **`grip_min_vacuum_pressure` is not a strength knob.** It's the threshold the driver uses for the "object detected at minimum pressure" enum on `object_detection_status`. The workspace doesn't override it; raising the max target does not change detection sensitivity.
- **Sim is unaffected.** Sim builds the same EPick `<ros2_control>` block with the fake driver. Its suction is the MuJoCo site behind `epick_controller`, which ignores the pressure params.
- **Inlining costs you the upstream defaults, not just the missing pass-through.** `grip_max_vacuum_pressure` is the parameter that motivated the inline, but every *other* parameter the wrapper would have set is now ours to supply too. `usb_port` is the one that mattered: left unset it silently resolved to `/dev/ttyUSB0`, which happened to be harmless while a separate PC owned the hardware and became a bring-up failure the moment the driver ran on the machine holding the tool-comms pty. When bumping the submodule, check what else the wrapper passes that this call does not.
- **The inlined call also passes `publish_gripper_joint_state`**, so the driver publishes the state of the revolute `gripper` joint after `epick_tcp`. See the submodule's [`policy_suction_channel.md`](../src/shelf_stocking/design_docs/policy_suction_channel.md).
- **Sim is insulated from the port by `use_dummy`**, which is bound to `use_fake_hardware` — the dummy driver never opens the device, so a wrong `usb_port` is invisible in MuJoCo and only ever surfaces on hardware.
- **Drift hazard with the inlined macro body.** If the upstream `epick_single_suction_cup` macro changes (new links, dimension changes, joint reshuffles), the inlined copy here will silently diverge. The CI build will catch broken XML, but it won't catch geometric drift. When bumping the `ros2_epick_gripper` submodule pointer, eyeball [`epick_single_suction_cup.xacro`](../src/external_dependencies/ros2_epick_gripper/epick_description/urdf/epick_single_suction_cup.xacro) against the inlined block in `picknik_009_ur5e.urdf.xacro` and patch any structural diff. The one intended difference is the `epick_tcp` to `epick_vacuum_state` joint: upstream it is fixed, here it is the revolute `gripper` joint. Keep it, or `ros2_control_node` aborts at startup; see the submodule's [`policy_suction_channel.md`](../src/shelf_stocking/design_docs/policy_suction_channel.md).
- **Verify it took effect at runtime.** The `epick_driver/EpickGripperHardwareInterface` logs `grip_max_vacuum_pressure: <value>kPa` at startup (`epick_driver/src/default_driver_factory.cpp`), so the easiest sanity check after a rebuild is to grep the moveit_studio container logs for that line.
