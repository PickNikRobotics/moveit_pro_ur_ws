# Vendored Camera Adapter Description

## What it does, at a glance

The wrist RealSense camera adapter bracket and the `realsense_d415` sensor macro are **vendored directly into `picknik_009_ur5e_config`** instead of being pulled from the `picknik_accessories` repo. The `picknik_accessories` git submodule has been removed from this workspace.

Vendored files (live under `picknik_009_ur5e_config`):

- `description/wrist_camera_adapter/picknik_ur_camera_adapter.urdf.xacro` — the `ur_realsense_camera_adapter` macro (bracket link + camera mount joints).
- `description/wrist_camera_adapter/realsense_d415.urdf.xacro` — the `realsense_d415` sensor macro (camera frames + Gazebo sensor block; Intel Apache-2.0 header preserved in-file).
- `meshes/wrist_camera_adapter/picknik_ur5_realsense_camera_adapter_rev2.dae` — adapter visual mesh.
- `meshes/wrist_camera_adapter/picknik_ur5_realsense_camera_adapter_rev2_collision.stl` — adapter collision mesh (also referenced by the sim's committed `mjcf/assets/robot_description.urdf`).

Both macros are included from `description/picknik_009_ur5e.urdf.xacro`. The config's `CMakeLists.txt` already installs `description/` and `meshes/` wholesale, so the new subfolders ship without a CMake change.

## Why this shape

`picknik_accessories` was a whole external repo (brackets, sensors, blender sources, mujoco assets, macros) of which this workspace used exactly four files — the wrist camera adapter pair plus their two meshes. Carrying a submodule for that surface added a clone/init step, a moving upstream pointer, and a CI submodule-SSH guard concern, all for assets that never change here. Vendoring the four files removes the submodule entirely and makes the config self-contained.

The mesh `package://` URIs inside the adapter macro and the sim URDF were repointed from `package://picknik_accessories/...` to `package://picknik_009_ur5e_config/meshes/wrist_camera_adapter/...`.

## Notes and gotchas

- **`realsense2_description` is now a direct dependency.** It is an apt package (`ros-humble-realsense2-description`), not a submodule, and was previously installed only because `picknik_accessories`'s `package.xml` declared it. `realsense_d415.urdf.xacro` includes `_materials.urdf.xacro` + `_usb_plug.urdf.xacro` from it. It is declared as an `exec_depend` in `picknik_009_ur5e_config/package.xml` — **do not drop it**, or the URDF fails to load. `robotiq_description` was added alongside it for the same reason (the config `$(find)`s it directly; it resolves from the `ros2_robotiq_gripper` submodule, but the direct consumer should declare it).
- **The `d415` camera mesh is decimated and config-local.** `meshes/d415.stl` was a 425k-triangle / 21 MB Catia export that tripped the MoveIt Pro UI's 10 MB-per-mesh URDF load gate (moveit_pro#19448). It is quadric-decimated to 85k triangles / ~4.1 MB (bounding box preserved within ~0.5%). **Both** d415 cameras reference this one repo copy: `description/scene_camera.urdf` (scene/bin camera, always pointed here) and `description/wrist_camera_adapter/realsense_d415.urdf.xacro` (wrist camera, repointed from `package://realsense2_description/meshes/d415.stl` so it no longer depends on the external apt mesh, which was the same unwieldy 21 MB file). The MuJoCo sim does not use the mesh at all — it renders a box proxy (see the comment in `picknik_009_ur5e_config_sim/mjcf/assets/additions.xml`) — so the unreferenced 21 MB `picknik_009_ur5e_config_sim/mjcf/assets/d415.stl` copy was removed.
- **`_sim` and `_hw_config` dropped their `picknik_accessories` dep** by inheritance: both depend on `picknik_009_ur5e_config`, which now owns the files. The sim's committed `mjcf/assets/robot_description.urdf` had its two collision-mesh URIs repointed.
- **Sim menagerie OBJ meshes were already local.** The `ur5e_visual/*.obj` split-by-color meshes in `mjcf/assets/` were copied in long ago; the only `picknik_accessories` trace there was a stale provenance comment, now corrected. Removing the submodule does not touch sim rendering — see [`sim_visuals.md`](sim_visuals.md).
- **Drift hazard:** these files no longer track `picknik_accessories` upstream. If the bracket geometry is revised upstream, the change must be re-vendored by hand.
