# picknik_009_ur5e_config

**This package is the base of an inheritance chain. Do not launch it on its own.**

To run on MuJoCo or real hardware, set `MOVEIT_CONFIG_PACKAGE` in `moveit_pro/.env` to one of the *child* configs and launch from there:

| To run on... | Set `MOVEIT_CONFIG_PACKAGE` to |
|---|---|
| MuJoCo simulation | `picknik_009_ur5e_config_sim` |
| Physical UR5e (Dev-009) | `picknik_009_ur5e_hw_config` |

See the [workspace README](../../README.md) for the full launch procedure.

## What this package provides

The shared, hardware-agnostic foundation that the sim and hw configs inherit from:

- **URDF / SRDF / meshes** for the UR5e + EPick gripper
- **MoveIt config** (kinematics, joint limits, joint groups, sensors_3d, jog)
- **ros2_control config** (`config/control/ros2_controllers.yaml`)
- **Objectives** common to both sim and hw
- **Waypoints** (`waypoints/ur_waypoints.yaml`)
- **Calibration data** (`objectives/calibration/`: the board calibration results and measured offsets)
- **Custom view panes** under `web/`

## Inheritance chain

```
picknik_009_ur5e_hw_config           ← real UR5e (use_fake_hardware: false)
  based_on: picknik_009_ur5e_config_sim
    based_on: picknik_009_ur5e_config   ← this package
```

Each child layer adds an objective-library path and overrides only the fields it needs (typically `hardware.robot_description.urdf_params`, `ros2_control.config`, and a few controllers).

## Why not run this standalone?

The `config/config.yaml` here sets `simulated: true` with `fake_hw_mode: "mock"` — ros2_control's pass-through mock driver, no physics, no MuJoCo. Those values exist so the inheritance default is sensible for the children to override, **not** because anyone runs the base config directly. There is no documented launch path, no `.env` example, and no CI coverage for launching it on its own. If you set `MOVEIT_CONFIG_PACKAGE=picknik_009_ur5e_config`, you are off the supported path.

If you want a fast no-physics smoke test of behavior trees, use the sim config — it's the lightest fully-supported launch mode.
