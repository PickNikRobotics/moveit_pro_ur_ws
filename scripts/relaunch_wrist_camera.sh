#!/usr/bin/env bash
# Relaunch the wrist_mounted_camera D415 inside the running drivers container, without
# restarting the stack, then wait for its colour frames. Run from the host.
#
#   scripts/relaunch_wrist_camera.sh           diagnose, then relaunch if that can help
#   scripts/relaunch_wrist_camera.sh --check   diagnose only (exit codes below)
#   scripts/relaunch_wrist_camera.sh --force   relaunch even if the camera is publishing
#
# Stops the wrist realsense2_camera_node the drivers launch started and starts
# pro_rs_launch.py again with the args of the wrist include in cameras.launch.xml (read
# from the file, so this never drifts from it). The colour-grid aligned_points node is left
# running: it only subscribes, and picks the new camera node up by itself. The relaunched
# node belongs to this script's launch, not the drivers launch: a drivers restart replaces
# it with the normal one.
#
# A relaunch fixes a RealSense node that crashed or lost its device and stayed dead. It
# cannot fix a camera that is off the USB bus (replug it), one that re-enumerated after the
# container started (the drivers container's /dev is a start-time snapshot: restart the
# drivers container), or a D415 whose USB link is failing (the kernel logs address and
# protocol errors on its port: move it to a port on another USB controller, then restart
# the drivers container). The script checks for those first and says which one it is. Never USB-reset a D415 in
# software: see src/shelf_stocking/design_docs/camera_watchdog.md.
#
# --check exit codes: 0 publishing, 1 stack not running, 2 a D415 is off the host bus,
# 3 a D415 is missing from the container's /dev, 4 not publishing but reachable (a
# relaunch can help), 5 the node started moments ago and may still be opening the camera,
# 6 the kernel logged USB link errors on a D415's port in the last few minutes (--force
# relaunches anyway, for when the link has since recovered).
#
# CALIBRATION_MODE=1 uses the calibration colour profile. WAIT_S sets how long to wait for
# frames. See design_docs/wrist_camera_recovery.md.
set -euo pipefail

CONTAINER="${CONTAINER:-moveit_pro-drivers-1}"
WAIT_S="${WAIT_S:-30}"
STARTUP_S=30
LINK_ERROR_WINDOW="-3 min"
# Kernel messages that mean the link to the device is failing, not a driver problem.
LINK_ERRORS="not accepting address|not responding to setup address|descriptor read|error -71|error -110|UVC probe control"
CAMERA=wrist_mounted_camera
TOPIC=/camera/$CAMERA/color/image_raw
LOG=/tmp/${CAMERA}_relaunch.log
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERAS_LAUNCH="$WS/src/picknik_009_ur5e_hw_config/launch/cameras.launch.xml"

MODE=relaunch
case "${1:-}" in
    "") ;;
    --check) MODE=check ;;
    --force) MODE=force ;;
    *) echo "usage: $0 [--check | --force]" >&2; exit 64 ;;
esac

# Read the wrist include's args from cameras.launch.xml, resolving the $(var ...) lets it
# uses for the chosen calibration mode. First line: the number of D415s the file launches.
mapfile -t LAUNCH_ARGS < <(python3 - "$CAMERAS_LAUNCH" "$CAMERA" "${CALIBRATION_MODE:-0}" <<'EOF'
import re, sys, xml.etree.ElementTree as ET
path, camera, calibration = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
root = ET.parse(path).getroot()
lets = {}
for let in root.iter("let"):
    cond_if, cond_unless = let.get("if"), let.get("unless")
    for cond in (cond_if, cond_unless):
        if cond not in (None, "$(var calibration_mode)"):
            sys.exit(f"{path}: let {let.get('name')} has an unhandled condition {cond}")
    if (cond_if and not calibration) or (cond_unless and calibration):
        continue
    lets[let.get("name")] = let.get("value")
d415s, wrist = 0, None
for inc in root.iter("include"):
    if not inc.get("file", "").endswith("/pro_rs_launch.py"):
        continue
    d415s += 1
    args = {a.get("name"): a.get("value") for a in inc.findall("arg")}
    if args.get("camera_name") == camera:
        wrist = args
if wrist is None:
    sys.exit(f"no {camera} include in {path}")
print(d415s)
for name, value in wrist.items():
    if name == "camera_name":
        continue  # the launch command passes it first, for the pkill pattern
    value = re.sub(r"\$\(var (\w+)\)", lambda m: lets[m.group(1)], value)
    if "$(" in value:
        sys.exit(f"{path}: cannot resolve {name}={value}")
    print(f"{name}:={value}")
EOF
)
EXPECTED_D415S=${LAUNCH_ARGS[0]}
LAUNCH_ARGS=("${LAUNCH_ARGS[@]:1}")
SERIAL=$(printf '%s\n' "${LAUNCH_ARGS[@]}" | sed -n "s/^serial_no:=//p" | tr -d "'\"")

docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true \
    || { echo "$CONTAINER is not running; start the stack first." >&2; exit 1; }

# Runs a command in the container as the host user, by name (docker applies the user's
# supplementary groups, video among them, only for a name), with the DDS settings of the
# drivers launch so it joins the same ROS graph.
ros_bash() {
    local flags=()
    if [ "$1" = "-d" ]; then flags=(-d); shift; fi
    docker exec "${flags[@]}" -u "$(id -un)" "$CONTAINER" bash -c "
        source ~/.moveit-pro-bashrc >/dev/null 2>&1
        export \$(tr '\0' '\n' < /proc/\$(pgrep -f ros2_control_node | head -1)/environ \
            | grep -E '^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)' | grep -v '=\$' | xargs)
        $1" "${@:2}"
}

# True when a frame arrives on $1 within 6 s.
publishing() {
    ros_bash "timeout 6 ros2 topic echo --once --no-arr $1 sensor_msgs/msg/Image" >/dev/null 2>&1
}

# The wrist node's pattern. Anchored so it does not match the
# wrist_mounted_camera_aligned_points node; bracketed so pgrep/pkill do not match the
# bash -c that runs them.
NODE_RE="[_]_node:=$CAMERA( |\$)"
node_age() {
    docker exec "$CONTAINER" bash -c "p=\$(pgrep -f '$NODE_RE' | head -1); [ -n \"\$p\" ] && ps -o etimes= -p \"\$p\" | tr -d ' '" || true
}

# Prints a diagnosis and returns the --check exit code.
diagnose() {
    echo "$CAMERA: D415 serial $SERIAL, topic $TOPIC"
    if publishing "$TOPIC"; then
        echo "  publishing."
        return 0
    fi
    echo "  NOT publishing."

    # Which D415s does the host see, at what speed, and does the container have each one's
    # device node? A RealSense serial is not readable from sysfs (its USB serial string is
    # different or empty), so a missing camera is told apart by count, not by serial.
    local present=0 missing_in_container=0 failing="" dev busnum devnum node speed port
    local klog_recent
    klog_recent=$(journalctl -k --since "$LINK_ERROR_WINDOW" --no-pager 2>/dev/null || true)
    for dev in /sys/bus/usb/devices/*; do
        [ "$(cat "$dev/idVendor" 2>/dev/null)" = 8086 ] || continue
        grep -qi "realsense" "$dev/product" 2>/dev/null || continue
        present=$((present + 1))
        busnum=$(printf %03d "$(cat "$dev/busnum")")
        devnum=$(printf %03d "$(cat "$dev/devnum")")
        node=/dev/bus/usb/$busnum/$devnum
        speed=$(cat "$dev/speed")
        local note=""
        [ "$speed" -lt 5000 ] && note="  <-- USB 2 (${speed} Mb/s): frames arrive in bursts; reseat it on a USB 3 port"
        # The container's /dev/bus/usb is a live bind mount, but its /dev/video* nodes are
        # a snapshot from when it started, so it is the video nodes that go missing.
        local vids missing_vids
        vids=$(ls -d "$dev"/*:*/video4linux/video* 2>/dev/null | xargs -rn1 basename | tr '\n' ' ')
        # shellcheck disable=SC2016
        missing_vids=$(docker exec "$CONTAINER" bash -c 'for v; do [ -c /dev/$v ] || echo -n "$v "; done' _ $vids)
        if [ -n "$missing_vids" ]; then
            missing_in_container=$((missing_in_container + 1))
            note="$note  <-- the container lacks ${missing_vids% } (it re-enumerated after the container started)"
        fi
        port=$(basename "$dev")
        if grep -qE "usb $port(:[0-9.]+)?: .*($LINK_ERRORS)" <<<"$klog_recent"; then
            failing="$failing $port"
            note="$note  <-- USB link errors on this port in the last few minutes"
        fi
        echo "  host: $(sed "s/ *$//" "$dev/product") at usb $port ($node), ${speed} Mb/s$note"
    done

    # Recent kernel USB trouble, the quickest pointer to a physical cause.
    local klog
    klog=$(journalctl -k --since "-30 min" --no-pager 2>/dev/null \
        | grep -E "USB disconnect|reset SuperSpeed|$LINK_ERRORS|HC died|xHCI host controller not responding" \
        | tail -n 5 || true)
    [ -n "$klog" ] && { echo "  recent kernel USB events:"; sed 's/^/    /' <<<"$klog"; }
    if grep -q "HC died\|not responding, assume dead" <<<"$klog"; then
        echo "  A USB host controller died. Restarting MoveIt Pro cannot fix that: unbind and rebind"
        echo "  it (src/shelf_stocking/design_docs/camera_watchdog.md), or reboot."
        return 2
    fi

    if [ "$present" -lt "$EXPECTED_D415S" ]; then
        echo "  Only $present of the $EXPECTED_D415S D415s are on the host USB bus."
        if publishing /camera/scene_camera/color/image_raw; then
            echo "  The scene camera is publishing, so the missing one is the wrist camera. Replug it"
        else
            echo "  The scene camera is not publishing either, so either one can be the missing one."
            echo "  Replug the one that dropped (lsusb cannot say which)"
        fi
        echo "  at the computer end and check its cable, wait until lsusb lists both D415s, then"
        echo "  restart the drivers container (docker restart $CONTAINER)."
        return 2
    fi
    if [ -n "$failing" ]; then
        local p bus ctrl
        for p in $failing; do
            bus=${p%%-*}
            ctrl=$(basename "$(readlink -f "/sys/bus/usb/devices/usb$bus/..")")
            echo "  The USB link to the D415 at usb $p is failing (kernel address/protocol errors), so"
            echo "  a relaunch cannot open it. It is on controller $ctrl, which also carries:"
            for other in /sys/bus/usb/devices/*; do
                [ -f "$other/idVendor" ] && [ "$other" != "/sys/bus/usb/devices/$p" ] || continue
                case "$(basename "$other")" in usb*) continue ;; esac
                [ "$(basename "$(readlink -f "/sys/bus/usb/devices/usb$(cat "$other/busnum")/..")")" = "$ctrl" ] || continue
                local prod
                prod=$(sed "s/ *$//" "$other/product" 2>/dev/null)
                [ -n "$prod" ] && echo "    usb $(basename "$other"): $prod"
            done
        done
        echo "  Move it to a USB 3 port on a different controller (lsusb -t shows which port is on"
        echo "  which bus), or try another cable, then restart the drivers container"
        echo "  (docker restart $CONTAINER). Do not USB-reset it from software."
        return 6
    fi
    if [ "$missing_in_container" -gt 0 ]; then
        echo "  A D415 re-enumerated after the drivers container started, so the container cannot"
        echo "  see it. Restart the drivers container (docker restart $CONTAINER); a relaunch cannot"
        echo "  reach it."
        return 3
    fi
    local age
    age=$(node_age)
    if [ -n "$age" ] && [ "$age" -lt "$STARTUP_S" ]; then
        echo "  The wrist node started ${age}s ago and may still be opening the camera."
        return 5
    fi
    echo "  Both D415s are on the bus and in the container: a relaunch of the wrist node can help."
    return 4
}

set +e
diagnose
STATUS=$?
set -e
[ "$MODE" = check ] && exit $STATUS
case $STATUS in
    0) [ "$MODE" = force ] || { echo "Nothing to do (--force relaunches anyway)."; exit 0; } ;;
    6) [ "$MODE" = force ] || exit $STATUS ;;
    2|3) exit $STATUS ;;
    5)
        AGE=$(node_age)
        echo "Waiting for it rather than restarting it..."
        for ((i = ${AGE:-0}; i < STARTUP_S; i += 5)); do
            sleep 5
            if publishing "$TOPIC"; then echo "$CAMERA is publishing."; exit 0; fi
        done
        echo "Still no frames; relaunching it."
        ;;
esac

echo "Stopping the running $CAMERA node..."
docker exec "$CONTAINER" bash -c "
    pkill -f '[p]ro_rs_launch.py camera_name:=$CAMERA' || true
    pkill -f '$NODE_RE' || true
    for _ in \$(seq 20); do pgrep -f '$NODE_RE' >/dev/null || exit 0; sleep 0.5; done
    pkill -9 -f '$NODE_RE' || true"
# Let librealsense release the device before the new node opens it.
sleep 2

echo "Relaunching (log: $LOG in $CONTAINER)..."
# camera_name first: the pkill above matches on it.
ros_bash -d "exec ros2 launch picknik_009_ur5e_hw_config pro_rs_launch.py camera_name:=$CAMERA \"\$@\" > $LOG 2>&1" \
    relaunch "${LAUNCH_ARGS[@]}"

echo "Waiting up to ${WAIT_S}s for frames on $TOPIC..."
for ((i = 0; i < WAIT_S; i += 5)); do
    sleep 5
    if publishing "$TOPIC"; then
        echo "$CAMERA is publishing again."
        exit 0
    fi
done

echo "No $CAMERA frames. Last lines of the relaunch log:" >&2
docker exec "$CONTAINER" tail -n 15 "$LOG" >&2 || true
if docker exec "$CONTAINER" grep -q "is NOT found" "$LOG" 2>/dev/null; then
    echo "The driver cannot find serial $SERIAL from inside the container. If lsusb lists both" >&2
    echo "D415s, restart the drivers container (docker restart $CONTAINER)." >&2
elif docker exec "$CONTAINER" grep -q -E "Frames didn't arrive|UVCIOC_CTRL_QUERY|Protocol error" "$LOG" 2>/dev/null; then
    echo "The camera opened but sends no frames: it is wedged. Replug it, then restart the" >&2
    echo "drivers container (the replug gives it new device nodes)." >&2
fi
exit 1
