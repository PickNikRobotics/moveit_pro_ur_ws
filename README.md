# moveit_pro_ur_ws

A [MoveIt Pro](https://picknik.ai/pro) 10 / Jazzy robot workspace for UR e-series arms. On this branch it runs the **Shelf Stocking Application** on the UR5e 009 cell: products picked from a bin onto a shelf, and unshelved from the shelf into a tote.

- **Robot configs:** `src/picknik_009_ur5e_config` (base), `_sim` (MuJoCo), `_hw_config` (hardware, EPick suction), `_hw_calibration_config`, and `picknik_009_ur5e_2f85_hw_config` (hardware with a Robotiq 2F-85 2-finger gripper). `src/ur_mock` / `src/ur_hw` are the generic UR configs from `main`, not yet migrated to Pro 10 and skipped by the build (`COLCON_IGNORE`).
- **Shelf-stocking Objectives and Behaviors:** the `src/shelf_stocking` submodule, shared by every robot and gripper config. See its README.

## Setup

```bash
git submodule update --init --recursive
```

Behaviors that call Gemini need `GOOGLE_GEMINI_API_KEY=<key>` in this workspace's `.env` (gitignored).

## Build and run

```bash
moveit_pro build -w <path to this workspace>
moveit_pro run -c picknik_009_ur5e_config_sim   # or _hw_config, or _2f85_hw_config
```

`-w` also makes this the CLI's active workspace. See [`design_docs/moveit_pro_10_build.md`](design_docs/moveit_pro_10_build.md).

## More

- Switching between the EPick and the 2F-85: [`design_docs/end_effector_variants.md`](design_docs/end_effector_variants.md).
- How the shared Objectives plug into a robot config: [`src/shelf_stocking/design_docs/robot_and_gripper_interface.md`](src/shelf_stocking/design_docs/robot_and_gripper_interface.md).
- Calibration: [`design_docs/board_based_calibration.md`](design_docs/board_based_calibration.md).
- UR hardware: the [UR Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/robot_arms/ur5e_hardware_setup_guide/).
