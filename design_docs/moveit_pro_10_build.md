# Moveit Pro 10 Build

## What it does, at a glance

On the `hw-migration-ten` line this workspace is compiled by the **system-installed MoveIt Pro 10 CLI** (`/usr/bin/moveit_pro`, from the deb; `/opt/moveit_pro/.version` names the exact release), not by a developer checkout of the `moveit_pro` repo:

```bash
moveit_pro build -w /path/to/this/workspace
```

That single command does two things in order: it builds the workspace's Docker images from [`Dockerfile`](../Dockerfile) at the repo root, then runs `colcon build` inside the resulting `base` image with the workspace bind-mounted. The Pro 9 flow this replaces — clone `moveit_pro`, `docker compose up dev-nvidia`, exec in, `colcon build` by hand — no longer applies, and neither does its `.env`.

Because the CLI is the build system now, most of what used to be workspace-side build configuration lives in `/opt/moveit_pro/` and in the CLI's own config file instead. This doc covers the parts of that arrangement that are surprising, and the one place where the workspace still had to change to accommodate it.

## Why this shape

### The workspace's `docker-compose.yaml` overrides only what it needs

The Pro 9 template was deleted. [`docker-compose.yaml`](../docker-compose.yaml) was later re-added with `drivers` mount overrides, a `runtime` dependency, and one service of its own, the IQ10's profile-gated `sam3_npu` sidecar.

The workspace used to ship the Pro 9 template. Every `docker compose` command the CLI issues layers the workspace's `docker-compose.yaml` *on top of* `/opt/moveit_pro/docker-compose.yaml`, so a workspace could override any service's environment, mounts, or devices without forking the installed file. The template that shipped with Pro 9 pre-declared each service with an empty body as a place to hang those overrides.

Pro 10 renamed the service set. The Pro 9 names (`agent_bridge`, `web_ui`) are simply not defined in the installed compose file any more, so merging the old template no longer *overrides* those services — it **declares new ones**, with empty bodies. Docker Compose rejects a service that has neither an image nor a build context, and it rejects the whole project when it does, so every CLI invocation failed at parse time, before any build work started:

```
service "web_ui" has neither an image nor a build context specified: invalid compose project
```

The file was deleted rather than rewritten with the Pro 10 service names. Nothing in it was ever overriding anything — all four bodies were empty — so a rewritten file would have been an empty placeholder carrying the same drift hazard forward into the next rename. MoveIt Pro's own workspace checker agrees: it flags a stale workspace compose file and offers to delete it. **If this workspace ever does need a compose override, re-add the file with only the service being overridden**, checked against the service names in the installed compose file at that time — not against a copied template.

The re-added file mounts two host directories into `drivers`. The live `/dev/bus/usb` gives the container the new node of a USB device that re-enumerates under a running stack, which the wrist camera recovery relies on ([`wrist_camera_recovery.md`](wrist_camera_recovery.md)). `/dev/serial` provides the stable path for the 2F-85's serial port; [`end_effector_variants.md`](end_effector_variants.md) explains why that is needed. Compose appends list values such as `volumes` on top of the installed file's lists, so restate nothing from the installed file. The `runtime` entry only adds a `depends_on` on `sam3_npu` with `required: false`, so the sidecar starts with the stack where the `sam3_npu` profile is active and the dependency disappears everywhere else. `sam3_npu` itself is a full service that `extends` the installed `base`; an empty-bodied service of its own would break the project the way the Pro 9 template did. See [`iq10_sam3_npu_sidecar.md`](iq10_sam3_npu_sidecar.md). Check every service it names against `/opt/moveit_pro/docker-compose.yaml` after each Pro upgrade. A compose-only change takes effect on the next `moveit_pro run`, with no rebuild.

### Distinct workspace directory names are load-bearing

The CLI derives `MOVEIT_HOST_USER_WORKSPACE_NAME` from the workspace's *directory name* and interpolates it into every image tag it builds (`moveit-pro-base:<version>-<distro>-<workspace name>`, and likewise for the runtime, drivers, and dev overlay images). Two checkouts of this repo in directories with the same basename therefore fight over the same tags, and each build silently invalidates the other's images.

So give every checkout of this repo on a host its own directory basename (for example one for a Pro 9 checkout and another for a Pro 10 one), and do not rename an existing checkout's directory casually: the rename orphans every image already built from it, and any git worktrees that hang off it record its absolute path and break until `git worktree repair` is run against each.

### Dependencies come from the image build, not from a manual step

The root `Dockerfile`'s base stage runs `rosdep install --from-paths src --ignore-src` against a bind-mount of the workspace source. Every `<depend>` in every `package.xml` under `src/` — including `moveit_studio_ur_pstop_manager`'s `ur_dashboard_msgs` and `ur_robot_driver` — is resolved and apt-installed while the image is being built, before `colcon` ever runs.

That is a real behavioral difference from the Pro 9 dev container, where the image was generic and the workspace's dependencies had to be installed by hand into a fresh container (an `install_ros_dependencies.bash` step run by hand). On this path that failure mode does not exist: adding a new `<depend>` is picked up by the next `moveit_pro build`, and *only* by a build that rebuilds the image — `moveit_pro build user_workspace` skips the image stage and will not see it.

## Notes and gotchas

### The Robotiq driver tracks upstream `main`

`ros2_robotiq_gripper` used to be pinned to its **Humble** branch, where `robotiq_driver`'s tests call a `hardware_interface::ResourceManager` constructor Jazzy no longer has. Because colcon aborts the whole run on a package failure, that one package was enough to leave `shelf_stocking_behaviors` and the rest of this workspace's own code unbuilt, so `robotiq_driver` and `robotiq_hardware_tests` were listed under `packages-ignore`.

Upstream `main` already guards that constructor on the `hardware_interface` version and targets Jazzy and later, so the submodule now tracks `main` and the two packages build and test again. The 2F-85 variant depends on the driver (see [`end_effector_variants.md`](end_effector_variants.md)). The EPick configs still use only `robotiq_description`'s `ur_to_robotiq_adapter` mounting-plate xacro.

Two consequences worth knowing. `colcon-defaults.yaml` is **copied into the image** by the `Dockerfile`, so a change to its `packages-ignore` takes an image rebuild (`moveit_pro build`, not `moveit_pro build user_workspace`) before `colcon` sees it. And `main` is not the Humble branch: it dropped the Hand-E description and renamed the 2F-85 macro's `sim_ignition` param to `sim_gazebo`, so callers of the macro change with the pin.

- **An unmigrated Pro 9 package breaks the image build.** The generic UR configs `ur_mock` and `ur_hw` from `main` were never migrated and have been removed; while they were here they needed a `COLCON_IGNORE`. Their `CMakeLists.txt` calls `moveit_studio_package()` from `moveit_studio_common`, which Pro 10 does not ship, and without the marker they fail the `Dockerfile`'s `rosdep install` with `Cannot locate rosdep definition for [moveit_studio_common]`, before any image exists. The marker works because both colcon and rosdep's package crawler (`catkin_pkg`) read it.
- **The `Dockerfile`'s base-image distro defaults to Jazzy**, as in `moveit_pro_empty_ws`. The CLI and CI pass `MOVEIT_ROS_DISTRO` explicitly, so the default only matters for a bare `docker build`. Before, it was Humble, and such a build pulled an image this workspace cannot compile against (see [`ci.md`](ci.md)).
- **`moveit_pro build -w` is not a per-invocation flag.** It *persists* the given path as the active workspace in the CLI's user config (`~/.config/moveit_pro/moveit_pro_config.9.yaml`), so every subsequent `moveit_pro run` / `dev` / `test` with no `-w` targets it too. Switching between the Pro 9 and Pro 10 checkouts on a shared machine means passing `-w` deliberately, or accepting that the last build wins.
- **The user config file keeps its `.9` name under Pro 10.** `moveit_pro_config.9.yaml` is the live file; the unsuffixed `moveit_pro_config.yaml` sitting next to it is the pre-9 legacy file and is ignored. Editing the wrong one is a silent no-op.
- **The GPU image variant is chosen by hardware probe on every invocation, not by config.** `custom_env()` in `moveit_pro_configuration.py` reads `MOVEIT_TARGET` from the user config and, finding nothing, calls `set_moveit_target()` — which probes for L4T, then NVIDIA, then AMD and returns the suffix that the ROS distro's image contract defines. So the suffix is *derived*, not stored: an empty `MOVEIT_TARGET` is the normal state, and a value in the config is an override rather than a cache. The same workspace on a GPU-less host pulls a different, smaller base image. Nothing in this repo selects it, and the suffix is versioned with the CUDA stack — so a base image tag that resolves today may not exist for a future Pro release, which surfaces as a pull failure rather than a config error. On a Jetson an override that disagrees with the required Jetson target is a hard error, not a silent downgrade.
- **The `Dockerfile`'s GPU stages are not what the GPU build uses.** `base-gpu` / `user-overlay-gpu` in the root `Dockerfile` install their own CUDA stack and hardcode a Python 3.10 (Ubuntu 22.04) library path. The compose services build the default final stage, so those stages are unreached dead code on this path — and their Python path is wrong for Jazzy's 24.04 base. Anything moved into them will not run until a compose `target:` points at them.
- **`.env` is read from the workspace directory now.** The CLI threads a `--env-file` for the workspace's own `.env` into every compose command, so `GOOGLE_GEMINI_API_KEY` and friends belong in a `.env` at the root of *this* repo — not in the `moveit_pro` source checkout's `.env`, which nothing on this path reads. It is gitignored; never commit a key.
- **Submodules in a git worktree are independent.** Git gives each worktree its own module directories, so `git submodule update --init --recursive` in a Pro 10 worktree checks out the Jazzy submodule pointers without disturbing the Humble ones in a sibling worktree. It is a full clone per worktree, though: the step needs network and is not instant. If a recorded submodule commit is not reachable from any branch on its origin, fetch it from the parent checkout's module directory rather than pushing a branch to work around it.
- **`moveit_pro configure` is not what makes the GPU image get pulled, and `-w` / `-c` make it do less than it looks like.** `ApplicationManager.configure` treats a configure as *full* only when it is handed no variables (`is_full_configure = len(kwargs) == 0`); passing `-w` or `-c` therefore writes exactly those keys and skips everything else, including the GPU report. Combined with the lazy `MOVEIT_TARGET` resolution above, a workspace that was only ever built still pulls the correct CUDA base image — the build log's `load metadata for docker.io/picknikciuser/moveit-pro:<tag>` line is the authoritative check, and grepping the user config for `MOVEIT_TARGET` is not evidence of anything. The reason to run a bare `moveit_pro configure` (no flags) is its networked Docker GPU-access test; `--skip-gpu-check` turns that test off for offline hosts and, by the same rule, does nothing at all on a `-w`/`-c` invocation.
- **The README's launch instructions are still the Pro 9 ones.** Bringing the runtime *up* under Pro 10 (`moveit_pro run`, the frontend, the custom view panes) has not been migrated or verified; only the build has. Treat a green build as saying nothing about whether the application runs.
