# Objective Library Precedence

## What it does, at a glance

When two Objective libraries loaded by one config ship a file with the same name, the config being run must get the variant meant for it. On MoveIt Pro 10 the one that wins is the library whose `objective_library_paths` id sorts **last alphabetically**, not the one nearest the leaf of the `based_on_package` chain. So the ids in this workspace's `config/config.yaml` files are named to sort in the order they should win:

| Config | Libraries it adds |
|---|---|
| `picknik_009_ur5e_config` (mock base) | the MoveIt Pro libraries (`core_`, `motion_`, `perception_`, `mujoco_objectives`), `w_shelf_stocking_objectives` (the submodule's shared `objectives/`), `x1_site_objectives` |
| `picknik_009_ur5e_config_sim` | `w_shelf_stocking_epick` (the submodule's `adapters/epick/`), `x2_sim_objectives` |
| `picknik_009_ur5e_hw_config` (gripper-neutral) | `x3_hw_objectives` |
| `picknik_009_ur5e_vacuum_config` | `w_shelf_stocking_epick`, `x4_vacuum_objectives` |
| `picknik_009_ur5e_2f85_hw_config` | `x4_2f85_objectives`, `y_shelf_stocking_robotiq_2f85` (the submodule's `adapters/robotiq_2f85/`) |

The resulting order is MoveIt Pro < shelf-stocking submodule (`w_`) < robot configs (`x1` … `x4`) < 2F-85 adapter (`y_`). The robot configs override a submodule file of the same name, and the submodule's files still win over a MoveIt Pro core file of the same name. The chain itself is described in [`end_effector_variants.md`](end_effector_variants.md); the gripper adapter contract is the submodule's [`robot_and_gripper_interface.md`](../src/shelf_stocking/design_docs/robot_and_gripper_interface.md).

## Why this shape

### The upstream bug

The runtime merges every config's `objective_library_paths` base → leaf and writes the result to `~/.config/moveit_pro/<config>/auto_created/complete_system_config.yaml` with `yaml.dump`. `yaml.dump` sorts keys by default, so that file lists the libraries alphabetically. The REST server's `sync_objective_library_links` (in `moveit_studio_rest_api/routers/objective_api/objective_common.py`) then reads the libraries back from that file in file order, applies "later library wins", and rewrites the symlinks in `~/.config/moveit_pro/<config>/objectives/`, which is the directory the objective server executes from. The Python `SystemConfigParser` that creates the links first gets the order right. The REST sync runs afterwards and replaces any link that points elsewhere, so the alphabetical order is what sticks.

With the original ids (`site_objectives`, `sim_objectives`, `hw_objectives`) the order came out inverted. On the hardware stack of that time, the base's `AlwaysSuccess` `Check Epick Grasp Status` passed every grip check, with or without a seal, and the sim's `Randomize Sim Products` failed calling a MuJoCo service that does not exist on hardware.

### Why the two adapter ids differ

The EPick adapter is `w_shelf_stocking_epick`, sorting **before** the robot `x…` ids; the 2F-85 adapter is `y_…`, sorting after them. The robot configs override one EPick adapter file, `Check Epick Grasp Status`: the sim's always succeeds, the vacuum config's reads the real EPick status. A `y_` id would let the adapter's default beat both, which is the same inversion as the bug above. No robot config overrides a 2F-85 adapter file, and the adapter's files are meant to win, so `y_` is safe there.

### Why rename rather than wait for the fix

Renaming the ids is invisible to everything else in the workspace: nothing references them by name. It also keeps working after upstream fixes the bug, because alphabetical order and inheritance order then agree anyway. The alternative, giving the variants distinct filenames, would put two files with the same BehaviorTree ID in the merged folder, and that resolves just as unpredictably.

## Notes and gotchas

- **A child cannot remove a parent's library, only replace it.** The merge keys libraries by id: a child that reuses an id replaces that entry, and a new id is added. So anything a parent loads lists under every child. That is why the hardware base is based on the mock config rather than the sim, and why the EPick adapter is loaded by the sim and the vacuum config rather than by a parent they share with the 2F-85.
- **The EPick adapter also sorts before the shared `objectives/` folder** (`w_shelf_stocking_epick` < `w_shelf_stocking_objectives`). No file name is in both today. An EPick adapter file named like a shared file would lose to it.
- **The sim and the hardware base are siblings.** Both have `Get Bin Image`, `Get Bin Point Cloud` and `Randomize Sim Products`, but no config loads both, so these are alternatives, not overrides. The same-named files that meet in one chain are `Check Epick Grasp Status` (adapter default, overridden by the sim or the vacuum config) and `Load Calibrated Scene` (base, overridden by the 2F-85).
- **Takes effect on a stack restart.** The merged config and the symlink folder are regenerated when the runtime starts (the folder is a tmpfs). Editing the ids on a running stack does nothing until then.
- **Verify by reading the links, not the source tree.** After a restart, `readlink ~/.config/moveit_pro/picknik_009_ur5e_vacuum_config/objectives/check_epick_grasp_status.xml` inside the runtime container must point into `picknik_009_ur5e_vacuum_config`. Under the 2F-85, `load_calibrated_scene.xml` must point into `picknik_009_ur5e_2f85_hw_config`.
- **Moving or deleting an Objective file leaves a dead link in `install/`.** With colcon's symlink install, `install/<package>/share/<package>/objectives/` holds a link per source file, and colcon never removes the link when the source goes away. The runtime links every entry of every library into the merged folder, dead ones too, so a dead link in a library that sorts later wins over the real file. Moving the EPick files from the shared `objectives/` into `adapters/epick` did exactly that: the stale shared `release_product.xml` link sorted after `w_shelf_stocking_epick`. After moving or deleting Objective files, delete that package's `build/` and `install/` folders and rebuild. On the host every link looks dead (they point at the container's `~/user_ws`), so look for dead links inside the container (`find install -xtype l`).
- **A new config package must keep the ordering.** A config based on `_hw_config` has to give any library it adds an id that sorts after `x3_hw_objectives`, as the `x4_…` ids do. A new gripper adapter with a file that a robot config overrides must sort before the `x…` ids, like `w_shelf_stocking_epick`.
- **Report it upstream.** The fix belongs in MoveIt Pro: dump the merged config with `sort_keys=False`, or have the sync use the parser's ordered list instead of re-reading the YAML.
