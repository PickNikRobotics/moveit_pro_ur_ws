# Control Loop Realtime Priority

Why the 500 Hz `controller_manager` loop overruns on this robot, what the overruns break,
and the two-part fix: the **lowlatency kernel** (the actual cause when it regresses) and
`thread_priority` (necessary, not sufficient).

## Symptom

The MoveIt Pro UI raises:

> **Recoverable Fault** — A fault was detected and a stop has been initiated. Press the
> 'Reset Arm' button to recover from this fault.

…on an arm that never protective-stopped. It shows up mid-objective (observed repeatedly
during `Shelf Stocking Main Application`), and the arm stops tracking: trajectories abort
with `Path tolerance violated on <joint> (deviation ~0.087 > tolerance 0.0873)` and teleop
refuses to move.

The giveaway is that the robot's own safety state is clean while the control program is
gone:

```
$ ros2 service call /dashboard_client/get_safety_mode ur_dashboard_msgs/srv/GetSafetyMode
  safety_mode=SafetyMode(mode=1), answer='Safetymode: NORMAL'      # 1 == NORMAL
$ ros2 service call /dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning
  answer='Program running: false', program_running=False
```

## Why the UI calls that a protective stop

It doesn't, quite — it calls it a *recoverable fault*, and
`moveit_studio_ur_pstop_manager`'s `publishFaultStatus()` raises that fault for **either**
a real p-stop **or** a stopped control program:

```cpp
// If the program is not running, indicate a recoverable fault, even if the robot is not
// in a protective stop state.
if (!isProgramRunning().value_or(true) && msg.status != NONRECOVERABLE_FAULT)
  msg.status = RECOVERABLE_FAULT;
```

So `Safetystatus: NORMAL` + `program_running: false` is indistinguishable, in the UI, from
a genuine protective stop. `Reset Arm` maps to `/recover_from_protective_stop`, which
unlocks the (nonexistent) p-stop, deactivates controllers, re-sends the program, and
reactivates the startup controller set.

Two traps when reading state by hand:

- **`programState` from the dashboard is not a health indicator.** In headless mode no
  `.urp` is loaded on the pendant, so it reads `STOPPED <unnamed>` even when the robot is
  perfectly healthy. Use `running` (dashboard) / `program_running` (service) instead.
- **`robot_fault_status` is `VOLATILE`** and event-driven, so a fresh subscriber sees
  nothing until the next transition — a tool that subscribes after the fault latched
  reads an empty topic, not a healthy one.

## The damage chain

`update_rate` is 500 Hz — a 2 ms budget, forced by the UR e-Series RTDE cap (see the
comment on the parameter itself). When the loop is late past the UR reverse-interface
watchdog, **the robot** hangs up and the driver merely reports it:

```
20:23:31.129  Connection to reverse interface dropped.
20:23:31.325  Successfully resent robot program            # driver auto-recovered once
20:23:31.361  Robot connected to reverse interface. Ready to receive control commands.
20:23:45.281  Connection to reverse interface dropped.     # second drop, stayed down
```

The driver sometimes re-sends the program itself and recovers silently; the fault only
latches when a drop isn't auto-recovered. **So the overruns are doing damage well before
anyone sees a banner** — treat a rising overrun count as the real alarm, not the UI.

## Root cause: the box booted the wrong kernel

This is the first thing to check, because it is the cause that has actually bitten, it is
invisible everywhere except `uname -r`, and no amount of scheduling tuning compensates
for it.

The Agent PC's host setup
requires **`linux-lowlatency`** (`CONFIG_PREEMPT=y`, `CONFIG_HZ=1000`) for the 500 Hz loop
and explicitly warns that stock generic — `PREEMPT_DYNAMIC` booted at
`preempt=voluntary` — "is not enough". It also flags the follow-up that was left undone:
`GRUB_DEFAULT` was never pinned to the lowlatency entry, so "a reboot or kernel update can
quietly land on generic".

That is exactly what happened. Observed 2026-09-09:

```
$ uname -r
6.8.0-139-generic                                   # generic, not lowlatency
$ grep -E '^CONFIG_PREEMPT=|^CONFIG_PREEMPT_VOLUNTARY=' /boot/config-$(uname -r)
CONFIG_PREEMPT_VOLUNTARY=y                           # no CONFIG_PREEMPT=y
$ grep ^GRUB_DEFAULT /etc/default/grub
GRUB_DEFAULT=0                                       # still unpinned
```

> **Do not reach for `/sys/kernel/realtime` as the check.** That file is created only by the
> `PREEMPT_RT` patch set, which this host deliberately does **not** run — the runbook picked
> `linux-lowlatency` over Ubuntu Pro's realtime kernel to keep the NVIDIA DKMS build. It is
> absent on a correctly configured box exactly as it is on a broken one, so it discriminates
> nothing, and `realtime_tools::has_realtime_kernel()` returns false here by design. `uname -r`
> and the `CONFIG_PREEMPT=y` grep are the only two checks that mean anything.

A generic `6.8.0-139` landed via apt, GRUB's "highest version wins" ordering preferred it
over the installed `6.8.0-138-lowlatency`, and the box has been running a
voluntary-preemption kernel ever since. On that kernel a `SCHED_FIFO` thread still waits
for in-kernel code to reach a voluntary preemption point, and with two RealSense USB3
streams and the NVIDIA driver on the same host those sections run tens of milliseconds.

Measured consequence, host idle, no objective running:

```
Overrun detected! The controller manager missed its desired rate of 500 Hz.
The loop took 48.939188 ms (missed cycles : 25).
```

**345 overruns/hour**, bimodal at ~2–6 ms and ~43–49 ms.

### The fix

`linux-image-6.8.0-138-lowlatency` is still installed, so this is a boot selection, not a
reinstall. Boot it and then pin GRUB so the next kernel update cannot undo it (the
runbook section above has the `GRUB_DEFAULT` string and the three post-reboot checks).
Verify with `uname -r` ending in `-lowlatency` and `CONFIG_PREEMPT=y`.

**Resolved 2026-09-09.** `GRUB_DEFAULT` is now pinned to the lowlatency entry by title, so an
apt kernel bump can no longer win the ordering on its own:

```
$ grep ^GRUB_DEFAULT /etc/default/grub
GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 6.8.0-138-lowlatency"
$ uname -r
6.8.0-138-lowlatency
$ grep -E '^CONFIG_PREEMPT=|^CONFIG_HZ=' /boot/config-$(uname -r)
CONFIG_PREEMPT=y
CONFIG_HZ=1000
$ nvidia-smi        # DKMS survived the swap - RTX 3060, driver 580.173.02
```

Two numbers are still owed, and **neither has been taken**: a post-fix overrun rate to set
against the 345/hour generic-kernel baseline, and the `cyclictest` baseline the runbook asks
for. Every overrun sample in this doc, the 345/hour figure included, is a generic-kernel
measurement. Record both the next time the stack runs a full objective — until then "fixed"
rests on the kernel being demonstrably correct, not on a demonstrated latency improvement.

### What was ruled out

Worth recording so nobody re-walks it. None of these were the cause:

| Suspect | Evidence against |
|---|---|
| Loop not realtime at all | It already was. `chrt -p <pid>` shows `SCHED_OTHER` because that is the **main** thread; the update loop is a separate thread (`ps -Lo tid,cls,rtprio`) and was `SCHED_FIFO` **50** before this change. |
| Work inside the loop (read/write/switch) | Only 2 `Overrun might occur` breakdowns in 2 h, and read/write were 24 µs / 35 µs. The rest are the loop waking *late*, not running long. |
| Controller switching | Real but rare: one 11.2 ms cycle, all of it `Activation time`, at startup. The recurring ~46 ms overruns happen with no switch within seconds and the arm idle. |
| EPick gripper on the loop | It *is* a synchronous 500 Hz component, but its `read()` only loads atomics set by a background thread — it does no serial I/O in the cycle. |
| cgroup CPU throttling | `cpu.max = max 100000`, `nr_throttled 0`. |
| Clock steps / NTP | chrony offset 0.8 ms, skew 0.011 ppm, no step events, `clocksource=tsc`. |
| Memory pressure / page faults | 39 GB available, no swap-in, PSI memory `full=0`, and `maj_flt = 0` for `ros2_control_node`. |
| Robot network | `enp111s0`: 0 errors, 0 dropped; ping to the arm 0.10/0.14/0.21 ms over 100 packets. |

## `thread_priority`: necessary, not sufficient

`controller_manager` 4.45.2 reads a `thread_priority` parameter and calls
`realtime_tools::configure_sched_fifo()`. Set in
`src/picknik_009_ur5e_hw_config/config/control/ros2_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    thread_priority: 80
```

The default is 50, which is the same priority as the kernel's `idle_inject/*` powerclamp
threads; 80 puts the control loop unambiguously above every non-migration RT thread on the
box. **Measured effect on its own: none** — 15 overruns in the 3 minutes after the change,
max 46.1 ms, statistically identical to before. Keep it anyway: it is the documented
ros2_control practice, and it is what lets the lowlatency kernel's preemption actually
deliver once the box is booted correctly. Do not mistake it for the fix.

No host-side change is required for it: the drivers container is already `privileged` with
an `rtprio=99:99` ulimit (see the runbook's "Realtime privileges are already handled"
section). Because `colcon-defaults.yaml` sets `symlink-install: true` and this YAML is
symlinked into `install/`, editing it takes effect on a **drivers restart with no
rebuild** (`docker restart moveit_pro-drivers-1` — the container owns `ros2_control_node`,
the `dashboard_client`, the p-stop manager and the controller spawners, so controllers
respawn on their own).

### Verifying it took

```
$ docker logs moveit_pro-drivers-1 2>&1 | grep -i "FIFO RT scheduling"
[ros2_control_node-6] Successful set up FIFO RT scheduling policy with priority 80.
$ ps -Lo tid,cls,rtprio,comm -p $(pgrep -f ros2_control_node) | awk '$2=="FF"'
1956747  FF  80  ros2_control_no
```

`chrt -p <pid>` is the wrong check — it reports the main thread and will say
`SCHED_OTHER` even when the loop is correctly `SCHED_FIFO`. The failure mode to watch for
is the sibling log line `Could not enable FIFO RT scheduling policy`, which means the
priority was requested but permission was missing.

### Deliberately not set

- **`lock_memory`** — supported, but `ulimit -l` inside the drivers container is only
  **8 MB**, so `mlockall` would fail. `maj_flt` is 0 anyway, so page faults are not the
  problem here.
- **`cpu_affinity`** — `realtime_tools::set_current_thread_affinity` is linked in but the
  parameter is not reliably exposed by name in this build. The runbook also warns against
  pre-emptively isolating CPUs on a 12-thread part. Revisit only with `cyclictest` numbers.
- **Lowering `update_rate`** — capped upward at 500 Hz by RTDE, and detuning the loop is
  not a fix for a kernel misconfiguration.

## Recovering a latched fault without the pendant

Headless mode is active, so the control program can be re-sent from software. Prefer the
recovery service over `resend_robot_program` alone — it deactivates controllers *first*,
which stops a controller from commanding a stale position when the program relaunches:

```
ros2 service call /recover_from_protective_stop std_srvs/srv/Trigger
```

[`scripts/reset_arm.sh`](../scripts/reset_arm.sh) does this from the host and reports the arm's state before and after. It refuses the stops software cannot clear (an e-stop or safeguard input, a safety fault, an arm that is powered off), because the service's unlock only retries for a while and then fails; the script says what to clear at the robot instead. Pro's fault status is published only when it changes and is not latched, so the script may not see it. When the UI shows a fault and the script reports a healthy arm, run it with `--force`. To call the service by hand, run it from inside `moveit_pro-drivers-1`. Note that the `ros2` CLI there discovers
nothing unless the node's DDS environment is re-exported first
(`CYCLONEDDS_URI=~/.ros/cyclonedds.xml`, interface `lo`, peer
`127.0.0.1`, multicast off) — without it `ros2 service list` returns zero lines and
`ros2 node info` reports "Unable to find node", while a `service call` on a name you
already know still works.

**Known issue, unrelated to this doc's fix:** the UI's own `Reset Arm` button has been
observed to fail ("Failed to reset protective stop") *without ever dispatching the
request* — the drivers container logged zero `Sending Unlock protective stop request`
lines across a whole session in which the button was pressed, while the same service
succeeded when called directly. The cause is in the Desktop App's own foxglove client,
not in this workspace. `foxglove_bridge` advertises every `std_srvs/Trigger` service
with an **empty request schema string**, because Trigger's request has no fields. The
client's `callService` rejects any service whose request schema is falsy ("No request
definition for service …") before sending anything, and the button handler turns that
string rejection into the generic message. So no Trigger-typed service can be called from the
UI, and `/recover_from_protective_stop` is one. A client fix also has to put a placeholder byte in
the request: the bridge answers a request carrying the CDR header plus the single
placeholder byte ROS 2 gives an empty message, and silently drops a header-only payload.
To check that a Pro upgrade has fixed it, read the `advertiseServices` entry for the
service on the bridge socket. If the button fails, call the service.
