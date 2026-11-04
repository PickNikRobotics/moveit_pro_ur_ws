# Sim vs HW Controller Tolerances

## At a glance

The `joint_trajectory_admittance_controller`'s `default_path_tolerance` is the only place in the three-way `ros2_controllers.yaml` split where sim and HW intentionally diverge on a tolerance value. The base config in
[`src/picknik_009_ur5e_config/config/control/ros2_controllers.yaml`](../src/picknik_009_ur5e_config/config/control/ros2_controllers.yaml) sets the tight value. The HW config in
[`src/picknik_009_ur5e_hw_config/config/control/ros2_controllers.yaml`](../src/picknik_009_ur5e_hw_config/config/control/ros2_controllers.yaml) re-declares the same tight value explicitly (i.e. it is not relying on inheritance — both files must be kept in sync when the HW-grade tolerance is changed). The sim config in
[`src/picknik_009_ur5e_config_sim/config/control/ros2_controllers.yaml`](../src/picknik_009_ur5e_config_sim/config/control/ros2_controllers.yaml) overrides it to a looser value.

Read the current numbers from those files — not from this doc.

## Why this shape

JTAC's path-tolerance check fires when the *measured* joint position drifts from the *commanded* trajectory point by more than the configured tolerance during execution. On real hardware the measured position closely tracks the reference once the UR5e's internal controller has converged, so a tight tolerance catches genuine following errors (cable snag, joint approaching limit, controller mistuned, payload heavier than modeled). In sim, MuJoCo's integrator, solver iteration counts, and tick cadence all introduce small numerical lag between the commanded reference and the simulated joint state that doesn't correspond to anything a HW controller would experience. The deviation is consistently small but not zero, and it can creep up to and past the HW-grade tolerance on otherwise nominal trajectories.

The trade-off:

- **Tighter sim tolerance** (closer to HW) — catches real bugs sooner if they exist, but produces spurious `Path tolerance violated` aborts on routine motions that are physically fine. Every false abort is a demo run lost and a misleading bug report.
- **Looser sim tolerance** (today's setting) — does not flag real-but-small following errors that would be caught on HW, but lets routine motions complete. We rely on HW runs (with the tighter base value) to catch the controller-following class of bugs.

We chose looser-in-sim because sim's purpose in this workspace is to iterate on Behavior Trees and perception, not to validate the controller. The controller validation happens on real hardware. There is no safety concern from a tolerance violation in sim — the worst case is a spurious abort.

## Notes and gotchas

- **HW re-declares the base value explicitly** — `picknik_009_ur5e_hw_config/config/control/ros2_controllers.yaml` carries its own `default_path_tolerance` line at the same tight value as the base file. Changes to the base do NOT propagate to HW automatically; both must be edited together. Tightening or loosening on HW is a safety-relevant change and should go through a deliberate review, not a "while I was in there" edit. **Future cleanup:** the redundancy is most likely a historical leftover (HW config was forked from base before the convention of inheriting non-overridden values stabilized) rather than an intentional decoupling — if a maintainer confirms that, the HW line can be deleted and HW will simply inherit. Until then, keep them in lockstep.
- **Sim is the only override** — if you find yourself thinking about adding a per-joint or per-trajectory tolerance override, check first whether the issue is really a controller-following bug or just sim noise. The base value is correct for HW; the sim value is the negotiated false-positive rate.
- **MTC trajectories go through JTAC too** — `ExecuteMTCSolutionDefault` and friends use the same controller, so this single setting governs every joint-space execution path including the shelf-stocking application's place branches. Bumping it in sim affects all of them, not just admittance subtrees.
- **The error message names the offending joint as of [moveit_pro #18971](https://github.com/PickNikRobotics/moveit_pro/pull/18971)** — `Path tolerance violated on shoulder_pan_joint (deviation 0.1014 > tolerance 0.1000)`. If you see this in sim, check the deviation against the configured sim tolerance before assuming a real bug; the gap is often single-millidegree numerical noise.
- **Not yet validated on HW after the most recent sim bump** — bumps in the sim file have no effect on HW (HW has its own explicit line in `picknik_009_ur5e_hw_config`), but if someone copies the looser value over to the base or to HW without thinking, HW behavior changes silently. Keep the override on the sim side only.
- **On HW, a deviation just over the tolerance usually means the arm stopped following, not that it lagged.** If the arm stops (a UR protective stop) or holds back (JTAC's admittance giving way to a carried product pressing on something), the error grows until it crosses the tolerance, and the abort reports the first sample over it, so the number always reads just above the limit. In one session, shoulder_pan aborts read a few ten-thousandths of a radian over at full speed and at the VLA collection's half speed alike, and one landed 30 ms after a `C157` (collision) from the UR. Check the drivers log for a UR `ErrorCodeMessage` and look at what the carried product could touch before slowing anything down. The main application's place transits and its `Look at Table` return were slowed slightly anyway (the `Look at Table` subtree takes `velocity_scale_factor` / `acceleration_scale_factor`, defaulting to full speed); that softens contact but does not prevent it.
