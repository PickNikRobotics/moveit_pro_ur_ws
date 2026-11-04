# Objective Library Precedence

## What it does, at a glance

The `6. HW/Sim Switching Subtrees` pattern depends on one rule: when `picknik_009_ur5e_config`, `_config_sim` and `_hw_config` all ship an Objective file with the same name, the config being run (the leaf of the `based_on_package` chain) wins. On MoveIt Pro 10 that rule only holds if the `objective_library_paths` ids in the three `config/config.yaml` files sort **alphabetically** in inheritance order. This is why they are named `x1_site_objectives` (base) < `x2_sim_objectives` < `x3_hw_objectives`, with an `x` prefix so they also sort after the `moveit_pro_objectives` libraries (`core_`, `motion_`, `mujoco_`, `perception_objectives`) that the base config pulls in.

Two more ids follow the same rule. The shared shelf-stocking Objectives from the `src/shelf_stocking` submodule are `w_shelf_stocking_objectives`. That sorts after the MoveIt Pro libraries but before `x1`, so the robot configs override a shared file of the same name, and the shared files still win over a MoveIt Pro core file of the same name. An end-effector variant config (`picknik_009_ur5e_2f85_hw_config`) adds `x4_2f85_objectives` (the gripper's generic Open/Close Gripper) and the application's adapter for that gripper, `y_shelf_stocking_robotiq_2f85` from the submodule, which sorts last so its gripper switching subtrees and favourite wrappers replace the shared EPick ones. See [`end_effector_variants.md`](end_effector_variants.md) and the submodule's [`robot_and_gripper_interface.md`](../src/shelf_stocking/design_docs/robot_and_gripper_interface.md).

## Why this shape

### The upstream bug

The runtime merges every config's `objective_library_paths` base → leaf and writes the result to `~/.config/moveit_pro/<config>/auto_created/complete_system_config.yaml` with `yaml.dump`. `yaml.dump` sorts keys by default, so that file lists the libraries alphabetically. The REST server's `sync_objective_library_links` (in `moveit_studio_rest_api/routers/objective_api/objective_common.py`) then reads the libraries back from that file in file order, applies "later library wins", and rewrites the symlinks in `~/.config/moveit_pro/<config>/objectives/`, which is the directory the objective server executes from. The Python `SystemConfigParser` that creates the links first gets the order right. The REST sync runs afterwards and replaces any link that points elsewhere, so the alphabetical order is what sticks.

With the old ids (`site_objectives`, `sim_objectives`, `hw_objectives`) the order came out as hw < sim < site, so it was inverted: the base config beat the sim config, and both beat hardware. On the HW stack this meant:

- `Check Epick Grasp Status` resolved to the base `AlwaysSuccess`, so every grip check passed, with or without a seal.
- `Randomize Sim Products` resolved to the sim version, which calls `ResetMujocoKeyframe`. The first objective to start with it failed with `Server not available for service /mujoco_system/reset_keyframe`.
- The bin and wrist camera subtrees resolved to the sim versions. The wrist ones subscribe to sim topic names.

### Why rename rather than wait for the fix

Renaming the ids is invisible to everything else in the workspace: nothing references them by name. It also keeps working after upstream fixes the bug, because alphabetical order and inheritance order then agree anyway. The alternative, giving the HW variants distinct filenames, would put two files with the same BehaviorTree ID in the merged folder, and that resolves just as unpredictably.

## Notes and gotchas

- **Takes effect on a stack restart.** The merged config and the symlink folder are regenerated when the runtime starts (the folder is a tmpfs). Editing the ids on a running stack does nothing until then.
- **Verify by reading the links, not the source tree.** After a restart, `readlink ~/.config/moveit_pro/picknik_009_ur5e_hw_config/objectives/check_epick_grasp_status.xml` inside the runtime container must point into `picknik_009_ur5e_hw_config`. Do the same for every file in `picknik_009_ur5e_hw_config/objectives/` that also exists in a parent config.
- **A new config package must keep the ordering.** A config based on `_hw_config` (for example `picknik_009_ur5e_hw_calibration_config`, which currently adds no library of its own) has to use an id that sorts after `x3_hw_objectives` if it ever adds one.
- **Report it upstream.** The fix belongs in MoveIt Pro: dump the merged config with `sort_keys=False`, or have the sync use the parser's ordered list instead of re-reading the YAML.
