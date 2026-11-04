# MuJoCo Sim Config

Conventions and tunables for the `picknik_009_ur5e_config_sim` MuJoCo scene that backs the simulated UR5e + e-Pick + workcell. Anything that affects how the physics engine steps, integrates, or contacts belongs here.

## At a glance

The sim is composed by `mjcf/scene.xml`, which `<include>`s several asset files. Physics options (`<option>`, `<flag>`) live in the **first** included file, `assets/robot_description.xml`; MuJoCo applies `<option>` from whichever file declares it, so do not duplicate `<option>` blocks across the includes.

```
mjcf/scene.xml
├── assets/robot_description.xml   ← <option> lives here
├── assets/ur_stand.xml
├── assets/additions.xml
│   └── assets/calibrated_scene.xml   ← generated at build time
│       └── assets/shelved_products.xml
└── assets/extra_objects.xml
```

For the actual integrator, timestep, viscosity, and flags in use today, read the `<option>` block in `assets/robot_description.xml` directly — it is the source of truth.

## Why physics options live in `assets/robot_description.xml`

`scene.xml` is a thin top-level composition file; the robot file owns the dynamics-relevant tags (joints, actuators, contacts). Keeping `<option>` adjacent to those means anyone touching the integrator or contact flags sees the timestep in the same view, instead of jumping to a separate scene file.

## Constraint islands and sleeping

Both `island="enable"` and `sleep="enable"` are set on the `<flag>` block. They are off-by-default in mainline MuJoCo and have to be turned on explicitly per scene; defaults left them dormant on this scene until 2026-05-08.

Why they make sense here:

- **Constraint islands** split the contact / equality-constraint graph into independent subgraphs ("islands") and solve each in isolation. This scene has 12 e-Pick `<weld>` constraints plus a swarm of small free-floating product bodies (`tomato_soup_can`, `tuna_fish_can`, `sugar_box`, …) that mostly do not touch each other — exactly the structure islands were designed for. The four `*_static_2` duplicate-stack bodies sit on top of their `*_static` parents, so each stacked pair shares one island; the remaining bodies stay isolated and still benefit from the partition. Without the flag, the solver works the entire constraint graph as a single block on every step.
- **Per-island sleeping** lets stationary islands skip integration entirely until something perturbs them. Most products on the table sit idle most of the time, so this is expected to drop per-step solver work substantially. Sleeping is a no-op without `island="enable"` — the two flags are paired.

`multiccd="enable"` was already on for stable suction-weld engagement on small geometries and is unaffected.

If a future regression makes the sim misbehave (small products jittering, welds engaging late), suspect these two flags first — try toggling them off in `assets/robot_description.xml` and re-running before chasing other causes.

## Tuning the timestep

This scene is **contact- and constraint-heavy** — multiple e-Pick `<weld>` equality constraints plus a swarm of small free-floating product bodies (`tomato_soup_can`, `tuna_fish_can`, `sugar_box`, …) referenced by the welds in `scene.xml`. That combination integrates badly at large step sizes: products fly off and suction welds destabilize. So this scene tolerates much smaller increases in `timestep` than the Franka or factory scenes shipped in `moveit_pro_example_ws`, and the timestep is held conservatively low even though that means `MujocoSystem` will sometimes log "not running in realtime" warnings on slower hosts (notably Jetson).

The realtime warning is **informational**, not fatal. Prefer accepting it over raising the timestep into the unstable regime.

If the warning becomes intolerable on a given host, try in this order:

1. Check whether **camera rendering** (RGBD sensors) is dominating the step budget — on Jetson that is usually the real bottleneck, not physics.
2. Disable or simplify any contacts/products you don't need for the current scenario.
3. Only as a last resort, bump `timestep` slightly. Validate that welds and products remain stable before committing.

## Notes and gotchas

- The `default` keyframe in `scene.xml` carries hand-tuned `qpos` for both arm joints and free-floating product bodies. Changing `timestep` does **not** require re-tuning the keyframe — `qpos` is positional, not rate-dependent.
- `MujocoSystem` is the `ros2_control` system plugin that owns the sim. Its realtime warning is emitted from inside the control loop, so a slow timestep manifests as control-loop jitter, not just visual stutter.
- The MoveIt Pro 3D Visualizer (not RViz) is the consumer for everything published from this sim.
- **`tcp_fts_sensor` site placement.** MuJoCo's `<force site>` / `<torque site>` measure the constraint force at the joint above the site's parent body. The site must sit on a body whose parent joint actually carries the EE load — for the UR5e that's `wrist_3_link` (with `wrist_3_joint`). Putting the site under an empty `ft_frame` body (no joint, no inertial, no descendants) makes the sensor read zero, which can present as `/force_torque_sensor_broadcaster/wrench` looking dead. Identity orientation on `wrist_3_link` aligns the data frame with `tool0`, matching `frame_id: tool0` and `sensor_frame: tool0` in `config/control/ros2_controllers.yaml`. The URDF's `ft_frame` link still exists via `ur_macro.xacro` and is published by `robot_state_publisher` regardless of the MJCF body layout, so removing the matching MJCF body is safe (the URDF→MJCF validator skips fixed joints).

## Joint position limits: MuJoCo mirrors HW, not the planner

MuJoCo's joint `range` (under each `<body>/<joint>`) and the matching `<position>` actuator `ctrlrange` (in the top-level `<actuator>` block) must mirror the **upstream UR5e URDF position limits** — i.e. the physical/HW range the real driver enforces — not the tighter values in `picknik_009_ur5e_config/config/moveit/joint_limits.yaml`. The YAML overrides are a planning-side convenience (asymmetric elbow / shoulder-lift ranges tuned for the bin-stocking workspace, ±180° wrists for OMPL connectedness); the sim should still be able to reach anywhere the real robot can, so that "I can do this on hardware" and "I can do this in sim" agree.

Both the joint `range` **and** the actuator `ctrlrange` need to be set: MuJoCo clamps the actuator's `ctrl` to `ctrlrange` *before* applying it, so a wider joint `range` with a narrow `ctrlrange` silently caps motion at `ctrlrange`. The tighter of the two always wins, and a mismatch between them is the most likely reason a joint appears to stop short of its declared range during teleop or scripted commands.

## The calibrated bodies are generated, the products are not

The tote (`calibrated_bin`), the shelving (`calibrated_top_shelf`, `calibrated_bottom_shelf`, `shelf_back`, `shelf_base` and the four flanking rows) and the two fixed cameras (`calibrated_camera`, `calibrated_shelf_cam`) come from `assets/calibrated_scene.xml`, which the package build generates from `cell_calibration.yaml` and `board_offsets.yaml`. They follow every calibration after one rebuild; never edit their poses by hand. See [`board_based_calibration.md`](board_based_calibration.md).

The free products are not generated. Their default poses in `assets/extra_objects.xml` and every keyframe in `keyframes.xml` are world positions laid out for the tote of an earlier calibration, so a calibration that moves the tote moves the walls around them.
- After a calibration that moves the tote, run `scripts/check_sim_products_in_bin.py` on the installed `mjcf/scene.xml`. It settles the default state and every keyframe and lists any product that ends outside the walls.
- A failure there is a product-layout fix in `keyframes.xml`: move the listed product toward the tote centre. The stacked duplicates (`*_static_2`) start well above the floor and land near the wall rims, so they are the usual cause; a mm-level tote move can flip one between staying in and falling out.
- Shifting every product by the tote's move keeps the layout relative to the tote but did not save the fragile stacked duplicate when tried, so it is not the default fix.

The table, workcell and camera mount plate are fixed in `additions.xml` and in `Load Calibrated Scene` alike.

The sim-only `overview_camera` site/camera in `additions.xml` (a far-away external viewpoint chosen for sim rendering) is **not** governed by the calibration. Don't sync it. Note this is the camera *previously* named `scene_camera`; the bin camera was renamed to `scene_camera` to match HW — see [`camera_naming.md`](camera_naming.md).
