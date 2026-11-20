# picknik_009_ur5e_config

**This package is the base of an inheritance chain. Do not launch it on its own.**

To run on MuJoCo or real hardware, run one of the leaf configs with `moveit_pro run -c <config>`:

| To run on... | Config |
|---|---|
| MuJoCo simulation (EPick suction) | `picknik_009_ur5e_config_sim` |
| Physical UR5e 009 with the Robotiq 2F-85 | `picknik_009_ur5e_2f85_hw_config` |
| Physical UR5e 009 with the EPick suction gripper | `picknik_009_ur5e_vacuum_config` |

See the [workspace README](../../README.md) for the full launch procedure.

## What this package provides

The shared foundation every other config inherits from:

- **URDF / SRDF / meshes** for the UR5e, with the `end_effector` xacro arg (EPick by default)
- **MoveIt config** (kinematics, joint limits, joint groups, sensors_3d, jog)
- **ros2_control config** (`config/control/ros2_controllers.yaml`): mock EPick controllers, which each runnable config replaces with its own
- **Objectives** common to every config (`Load Calibrated Scene`, `Look at Table`, `Reset And Look at Table`, the board placement subtrees)
- **Waypoints** (`waypoints/ur_waypoints.yaml`)
- **Calibration data** (`objectives/calibration/`: the board calibration results and measured offsets)

## Inheritance chain

```
picknik_009_ur5e_config                  ← this package (mock)
├── picknik_009_ur5e_config_sim          ← MuJoCo, EPick
└── picknik_009_ur5e_hw_config           ← real UR5e, gripper-neutral (not run on its own)
    ├── picknik_009_ur5e_vacuum_config   ← EPick suction
    └── picknik_009_ur5e_2f85_hw_config  ← Robotiq 2F-85
        └── picknik_009_ur5e_2f85_policy_config
```

Each child layer adds objective-library paths and overrides only the fields it needs (typically `hardware.robot_description.urdf_params`, `ros2_control`, and its drivers launch). Why the hardware base sits on this package rather than on the sim, and which Objectives live where, is in [`design_docs/end_effector_variants.md`](../../design_docs/end_effector_variants.md).

## Why not run this standalone?

The `config/config.yaml` here sets `use_fake_hardware: true` with `fake_hw_mode: "mock"` — ros2_control's pass-through mock driver, no physics, no MuJoCo. Those values exist so the inheritance default is sensible for the children to override, **not** because anyone runs the base config directly. It also loads no gripper adapter, so some gripper switching subtrees do not resolve. If you run `picknik_009_ur5e_config`, you are off the supported path.

If you want a fast no-physics smoke test of behavior trees, use the sim config — it's the lightest fully-supported launch mode.
