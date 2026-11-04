# Known Benign Warnings

Startup/runtime warnings that look alarming but are upstream cosmetic issues, not bugs in this workspace. Before chasing one of these, check this list — if your warning matches verbatim, you can ignore it (or file an upstream issue against the package named below).

## `rcl.logging_rosout`: "Publisher already registered for provided node name"

### What it looks like

```
[component_container_mt-7] [WARN] [<ts>] [rcl.logging_rosout]: Publisher already registered for
provided node name. If this is due to multiple nodes with the same name then all logs for that
logger name will go out over the existing publisher. As soon as any node with that name is
destructed it will unregister the publisher, preventing any further logs for that name from
being published on the rosout topic.
```

You see it from `[component_container_mt-7]` — that's MoveIt Pro's `moveit_studio_container` (a multi-threaded `rclcpp_components` container, started in `moveit_pro/src/agent/moveit_studio_agent/launch_description.py` at the `ComposableNodeContainer(name="moveit_studio_container", executable="component_container_mt", ...)` call).

Typical observed pattern in our launch logs:
- 1× warning during startup, between `Loaded node '/mtc_task_manager_node'` and `Loaded node '/planning_scene_listener_node'`.
- 4× warnings in a tight burst ~37 s into startup, immediately before `controller_manager` configures controllers.

### Why it fires

Several composable nodes inside `moveit_studio_container` (`mtc_task_manager_node`, `planning_scene_listener_node`, `srdf_publisher`, `robot_state_publisher`) construct **internal helper `rclcpp::Node`s** during their constructors — for the `PlanningSceneMonitor`, `RobotModelLoader`, `SynchronizedStringParameter`, etc. Some of those helpers compute their internal node names from the parent name plus a fixed suffix, with **no per-instance disambiguator**. When two helpers in the same container process land on the same name (or race during the controller-spawn flurry), the second one collides with the first on rosout publisher registration, and rcl emits this warning.

MoveIt Pro is partially aware of this: `moveit_pro/src/moveit_legacy/moveit_pro_ros/planning_scene_monitor/src/planning_scene_monitor.cpp:117-125` already adds a per-instance `_private_<this-pointer>` suffix to the PSM's helper node and explicitly comments **"Add random ID to prevent warnings about multiple publishers within the same node."** The remaining warnings come from the helpers that didn't get the same fix — notably `moveit_pro/src/moveit_legacy/moveit_pro_base/rdf_loader/src/synchronized_string_parameter.cpp:124`, which uses a bare `<parent_name>_ssp_<param_name>` with no per-instance suffix.

### Impact

Cosmetic. The warning message itself spells out the actual behavior: rosout for the duplicated logger name keeps flowing through whichever publisher registered first. No log messages are lost.

### Where the fix belongs

**Upstream in `moveit_pro`, not here.** Extend the `_private_<this-pointer>` pattern from `planning_scene_monitor.cpp:120` to the other helper-node creation sites (start with `synchronized_string_parameter.cpp:124`).

**Do not** "fix" it in `shelf_stocking_ws` by splitting `moveit_studio_container` into separate processes — you'd lose intra-process zero-copy comms and gain nothing real, since the warning is harmless.
