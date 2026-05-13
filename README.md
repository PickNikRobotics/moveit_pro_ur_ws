# moveit_pro_ur_ws

This is a [MoveIt Pro](https://picknik.ai/pro) robot workspace for UR e-series robots.

Refer to the [UR Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/robot_arms/ur5e_hardware_setup_guide/) for installation.

## Build Instructions

This repository uses git submodules. Clone with:
```bash
git clone --recurse-submodules <repo-url>
```

If you already cloned without submodules, initialize them with:
```bash
git submodule update --recursive --init
```

---

```bash
moveit_pro configure
moveit_pro build
moveit_pro run -v picknik_007_rail_hw
```

## Instructions for Running Hardware

### picknik_007_rail_hw

This is the PickNik configuration for the following combination of hardware
- Vention Machine Motion 2 linear rail with E-Stop
- UR 5e AKA 7e with E-Stop
- Robotiq 2F-85 connected via tool communication port
- Wrist mounted Realsense via PickNik [wrist adapter plate](https://github.com/PickNikRobotics/picknik_accessories/tree/fcdb1ff8b729c93dbd5f15a09c33098b32caeac5/descriptions/brackets/ur_realsense_camera_adapter)
- Single PC compute

1. Connect all hardware components.
2. Reset any closed E-Stop buttons.
3. The 007 linear rail does not have global encoders and must re-zero each time the ROS driver is restarted. Visually verify the IGUS chain and UR will be clear of collisions when the rail moves to both limits.
  - If the UR should be adjusted, you can do this via the UR pendant Freedrive mode.
4. The `Machine Motion 2` controller box `Status LED` should either be white (ready to boot) or red (booted and ready for ROS 2 driver to be launched) or green (booted and ROS 2 driver has been launched since boot). Full `Status LED` definitions are listed [here](https://docs.vention.io/docs/machinemotion-controller-manual).
  - If the `Status LED` is white, press the power button and wait for the `Status LED` to transition to red.
5. The UR should be started and put into Remote control mode via the pendant.
  - Power on pendant
  - `Power off` (bottom left)
  - `ON`
  - `START`
  - `Exit`
  - `Local` (top right, only if in Local mode) ---> `Remote Control`
6. The next step will cause the rail to re-zero
7. `moveit_pro run -v -c picknik_007_rail_hw`
