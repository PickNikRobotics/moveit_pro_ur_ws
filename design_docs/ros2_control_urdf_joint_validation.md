# Ros2 Control Urdf Joint Validation

## What it does, at a glance

An invariant that became load-bearing when this workspace moved to MoveIt Pro 10 / Jazzy:
**every `<joint name="...">` inside every `<ros2_control>` block must name a joint that actually
exists in the assembled URDF.** Under ros2_control 4.x a name with no match is a startup abort,
not a warning. Humble ignored it, so any config carried over from that era may be sitting on a
dangling reference it has never been punished for.

Nothing in this workspace currently violates it. This doc exists because the failure is very hard
to read backwards from its symptom, and because the next `<ros2_control>` block someone adds is
one typo away from reproducing it.

## Failure semantics

This is the whole reason the page exists. `parse_control_resources_from_urdf()` **throws**, the
exception is never caught, and the process aborts:

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  Joint 'gripper' not found in URDF
Aborted (Signal sent by tkill() ...)
[ERROR] [ros2_control_node-1]: process has died [pid ..., exit code -6, ...]
```

What actually reaches the operator is `exit code -6` (SIGABRT) and a 19-frame stack trace,
followed by every controller spawner being SIGINT'd and the `drivers` container exiting 1.
**Nothing in that cascade names the URDF**, and the `what():` line scrolls past hundreds of lines
before the visible failure. The spawner deaths are collateral from a node that already died.

**Triage rule:** when `drivers` exits 1, grep for the exception before reading anything else.

```bash
grep -n "terminate called\|what():" ~/.config/moveit_pro/last_moveit_pro_run.log
```

## Verifying the invariant without starting the robot

A full `moveit_pro run` cycle is an expensive way to find out you mistyped a joint name. The same
check runs offline against the expanded URDF, in the base image with the workspace mounted:

```bash
xacro install/picknik_009_ur5e_config/share/picknik_009_ur5e_config/description/picknik_009_ur5e.urdf.xacro \
  name:=ur5e use_fake_hardware:=true fake_hw_mode:=mujoco robot_ip:=0.0.0.0 has_tool_changer:=false \
  > /tmp/out.urdf
```

then cross-check with a few lines of ElementTree: collect
`{j.get("name") for j in root.findall("joint")}`, walk `root.findall("ros2_control")`, and report
any child `<joint>` whose name is not in that set. A healthy sim expands to 37 URDF joints, with
the six `ur_mujoco_ros2_control` arm joints resolving and the EPick block's `gripper` resolving to the revolute `epick_tcp` to `epick_vacuum_state` joint.

## Notes and gotchas

- **The EPick block declares `<joint name="gripper">` only on request.** `epick_driver_ros2_control.xacro` emits it when the caller passes `publish_gripper_joint_state="true"`. This workspace does, and its URDF carries the matching revolute `gripper` joint, which exists to put the grip state on joint states (see the submodule's [`policy_suction_channel.md`](../src/shelf_stocking/design_docs/policy_suction_channel.md)). The opt-in and the URDF joint must change together: dropping the joint while the call still opts in reproduces the abort. The SRDF's `gripper` **group** is an unrelated link chain in a different namespace.
- **A stale submodule checkout runs older submodule code.** `git submodule update` is not automatic on branch checkout, so a workspace that skipped it runs an older submodule than the superproject records. For `ros2_epick_gripper` that fails at xacro, before `ros2_control_node` starts, with `Invalid parameter "publish_gripper_joint_state"`, because this workspace passes the opt-in. If `git status` shows `M src/external_dependencies/<name>` with no local edits, the checkout is at a different commit than the superproject records: run `git submodule update --init --recursive` before diagnosing anything downstream of it, and check the direction with `git diff --submodule=log`, which prints `(rewind)` when the working checkout is *behind*.
- **`--` is illegal inside an XML comment, and xacro reports it from the wrong altitude.** Writing
  an em-dash-style `--` in a comment fails as `XML parsing error: not well-formed (invalid token)`
  at a line and column *inside the comment*, which reads like a structural problem with the
  surrounding markup.
- **`$(arg name)` has no default in this URDF.** Expanding it by hand requires `name:=<anything>`;
  omitting it fails with `Undefined substitution argument name`, which does not identify the
  missing argument except by naming it literally.
