#!/usr/bin/env bash
# Watch the scene_camera D415 and bring it back by itself when it drops. Run from the host;
# long-running, Ctrl-C stops it. Made for a host whose USB hub resets and drops the cameras
# (iq9-1): the D415 re-enumerates with new /dev/video* numbers, the drivers container's /dev
# is a snapshot from container start, and the RealSense V4L2 backend cannot reopen it.
#
# It notices an outage within a few seconds, two ways, both checked every POLL_S for free:
#   - the D415's USB device number changed (it re-enumerated);
#   - no frame for STALL_S: a small subscriber inside the drivers container listens to the
#     scene camera's color/camera_info (published with every frame, a few hundred bytes) and
#     touches FRAME_FILE, which sits in a directory the container shares with the host.
# Recovery: wait until the D415 and its video devices have been on the bus without a kernel
# reset for SETTLE_S, stop the scene_camera node, copy the host's current /dev/video* and
# /dev/media* nodes into the privileged drivers container, start the scene_camera include of
# cameras.launch.xml again (args read from the file), and wait for frames. A D415 often gets
# renumbered again while its node opens it, so a failed attempt is retried with backoff
# until frames come back. The relaunched node belongs to this script's launch; a drivers
# restart replaces it with the normal one, and this script then watches that one.
#
# It never restarts the drivers container (that drops arm control) and never USB-resets the
# camera itself. It survives stack restarts. See design_docs/scene_camera_recovery.md.
#
# Env: CONTAINER, POLL_S, STALL_S, SETTLE_S, FRAME_WAIT_S, CALIBRATION_MODE=1 (the
# calibration colour profile), SCENE_CAMERA_SERIAL (default: the launch file's default),
# WS (workspace with cameras.launch.xml; default: this script's workspace).
set -uo pipefail

CONTAINER="${CONTAINER:-moveit_pro-drivers-1}"
POLL_S="${POLL_S:-1}"
STALL_S="${STALL_S:-3}"
SETTLE_S="${SETTLE_S:-1}"
FRAME_WAIT_S="${FRAME_WAIT_S:-10}"
STARTUP_GRACE_S=30
CAMERA=scene_camera
TOPIC=/camera/$CAMERA/color/image_raw
INFO_TOPIC=/camera/$CAMERA/color/camera_info
D415_ID=8086:0ad3
WS="${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CAMERAS_LAUNCH="$WS/src/picknik_009_ur5e_hw_config/launch/cameras.launch.xml"
NODE_RE="[r]ealsense2_camera_node.*__node:=$CAMERA( |\$)"
USER_NAME="$(id -un)"
HELPER=/tmp/${CAMERA}_monitor_helper.sh
# The running node's command line, saved while it is healthy, with its --params-file
# arguments pointing at saved copies: recovery starts the node binary directly from it,
# which skips the seconds ros2 launch takes to start on this board.
NODE_CMD=/tmp/${CAMERA}_node_cmdline
FRAME_WATCH=/tmp/${CAMERA}_frame_watch.py
LOG=/tmp/${CAMERA}_relaunch.log
# The same directory from both sides: the drivers container mounts the host's
# ~/.ros/log_moveit_pro at ~/.ros/log.
FRAME_FILE_HOST="$HOME/.ros/log_moveit_pro/${CAMERA}_last_frame"
FRAME_FILE_CONTAINER="/home/$USER_NAME/.ros/log/${CAMERA}_last_frame"
# The wrist Orbbec (see wrist_loop).
WRIST_STALL_S="${WRIST_STALL_S:-5}"
WRIST_RETRY_S="${WRIST_RETRY_S:-15}"
WRIST_STARTUP_GRACE_S="${WRIST_STARTUP_GRACE_S:-30}"
ORBBEC_ID=2bc5:0840
WRIST_FILE_HOST="$HOME/.ros/log_moveit_pro/wrist_orbbec_last_frame"
WRIST_FILE_CONTAINER="/home/$USER_NAME/.ros/log/wrist_orbbec_last_frame"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# The scene_camera include's args from cameras.launch.xml, one name:=value per line, with
# $(var ...) resolved from the file's arg defaults and its calibration_mode lets.
read_launch_args() {
    python3 -I - "$CAMERAS_LAUNCH" "$CAMERA" "${CALIBRATION_MODE:-0}" "${SCENE_CAMERA_SERIAL-}" <<'EOF'
import re, sys, xml.etree.ElementTree as ET
path, camera, calibration = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
serial = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] else None
root = ET.parse(path).getroot()
values = {a.get("name"): a.get("default") for a in root.iter("arg") if a.get("default") is not None}
if serial is not None:
    values["scene_camera_serial"] = serial
for let in root.iter("let"):
    cond_if, cond_unless = let.get("if"), let.get("unless")
    if cond_if == "$(var calibration_mode)" and not calibration:
        continue
    if cond_unless == "$(var calibration_mode)" and calibration:
        continue
    if (cond_if or cond_unless) and "$(var calibration_mode)" not in (cond_if, cond_unless):
        continue  # not one this script can resolve; fails below only if the include uses it
    values[let.get("name")] = let.get("value")
for inc in root.iter("include"):
    if not inc.get("file", "").endswith("/pro_rs_launch.py"):
        continue
    args = {a.get("name"): a.get("value") for a in inc.findall("arg")}
    if args.get("camera_name") != camera:
        continue
    for name, value in args.items():
        if name == "camera_name":
            continue  # passed first by the launch command
        def sub(m):
            if m.group(1) not in values:
                sys.exit(f"{path}: cannot resolve $(var {m.group(1)}) for {name}")
            return values[m.group(1)]
        value = re.sub(r"\$\(var (\w+)\)", sub, value)
        if "$(" in value:
            sys.exit(f"{path}: cannot resolve {name}={value}")
        print(f"{name}:={value}")
    sys.exit(0)
sys.exit(f"no {camera} include in {path}")
EOF
}

container_running() {
    docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true
}

# A restarted stack replaces the container under the same name, and its /tmp helpers and
# frame watchers with it, so each loop pins the instance it set up and starts over when the
# instance changes, not only when the container is gone.
container_instance() { docker inspect -f '{{.Id}} {{.State.StartedAt}}' "$CONTAINER" 2>/dev/null; }
# Seconds since the drivers container started (for start-up grace).
container_age() {
    local t
    t=$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER" 2>/dev/null) || { echo 0; return; }
    echo $(($(date +%s) - $(date -d "$t" +%s 2>/dev/null || date +%s)))
}
same_container() { [ -n "${CUR_INSTANCE:-}" ] && [ "$(container_instance)" = "$CUR_INSTANCE" ] && container_running; }

# "port#devnum#videonodes" of the D415 on the host bus, empty if it is not there.
d415_usb_id() {
    local d v
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] || continue
        [ "$(cat "$d/idVendor"):$(cat "$d/idProduct")" = "$D415_ID" ] || continue
        v=$(ls -d "$d"/*/video4linux/video* 2>/dev/null | wc -l)
        echo "$(basename "$d")#$(cat "$d/devnum")#$v"
        return
    done
}

# Seconds since the last scene camera frame (large if none yet).
frame_age() {
    local t
    t=$(stat -c %Y "$FRAME_FILE_HOST" 2>/dev/null) || { echo 999999; return; }
    echo $(($(date +%s) - t))
}

# Writes the in-container helper and frame watcher (both are lost with the container's /tmp).
install_helpers() {
    docker exec -i "$CONTAINER" bash -c "cat > $HELPER && chmod 755 $HELPER" <<EOF
#!/usr/bin/env bash
source ~/.moveit-pro-bashrc >/dev/null 2>&1
export \$(tr '\0' '\n' < /proc/\$(pgrep -f ros2_control_node | head -1)/environ \\
    | grep -E '^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)' | grep -v '=\$' | xargs)
case "\$1" in
    launch) shift; exec ros2 launch picknik_009_ur5e_hw_config pro_rs_launch.py \\
        camera_name:=$CAMERA "\$@" > $LOG 2>&1 ;;
    node) mapfile -d '' -t cmd < $NODE_CMD; exec "\${cmd[@]}" > $LOG 2>&1 ;;
    watch) shift; exec python3 $FRAME_WATCH "\$@" ;;
    call) shift; exec timeout 15 ros2 service call "\$@" ;;
esac
EOF
    docker exec -i "$CONTAINER" bash -c "cat > $FRAME_WATCH" <<'EOF'
# Touch a file whenever the camera publishes a frame's camera_info, at most twice a second.
import os, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

topic, path, name = sys.argv[1], sys.argv[2], sys.argv[3]
last = 0.0

def on_info(_msg):
    global last
    now = time.monotonic()
    if now - last < 0.5:
        return
    last = now
    with open(path, "a"):
        pass
    os.utime(path)

rclpy.init()
node = rclpy.create_node(name)
node.create_subscription(CameraInfo, topic, on_info, qos_profile_sensor_data)
rclpy.spin(node)
EOF
}

frame_watch_running() {  # topic
    docker exec "$CONTAINER" pgrep -f "[p]ython3 $FRAME_WATCH $1 " >/dev/null 2>&1
}

# Start a frame watcher for a camera_info topic unless one runs. Default: the scene camera.
ensure_frame_watch() {  # [topic container_file node_name]
    local topic=${1:-$INFO_TOPIC} file=${2:-$FRAME_FILE_CONTAINER} name=${3:-scene_camera_frame_watch}
    frame_watch_running "$topic" && return
    docker exec -d -u "$USER_NAME" "$CONTAINER" "$HELPER" watch "$topic" "$file" "$name"
}

file_age() {  # host path -> seconds since it was touched (large if never)
    local t
    t=$(stat -c %Y "$1" 2>/dev/null) || { echo 999999; return; }
    echo $(($(date +%s) - t))
}

usb_present() {  # vid:pid
    local d
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor"):$(cat "$d/idProduct")" = "$1" ] && return 0
    done
    return 1
}

# The wrist Orbbec's watch topic and reboot service, read from the drivers launch's own
# orbbec_stream_watchdog (so they follow cameras.launch.xml, dual colour or not), which is
# then stopped: two watchdogs rebooting one camera lengthen every outage. A drivers restart
# starts it again, and the next call stops it again. Cached for a restart of this script.
wrist_config() {
    docker exec "$CONTAINER" bash -c '
        cfg=/tmp/wrist_orbbec_monitor_cfg
        p=$(pgrep -f "[o]rbbec_stream_watchdog.py" | head -1)
        if [ -n "$p" ]; then
            files=$(tr "\0" "\n" < /proc/$p/cmdline | grep -A1 -- --params-file | grep -v -- "^--")
            topic=$(grep -h "camera_info_topic:" $files 2>/dev/null | tail -1 | awk "{print \$2}" | tr -d "\"'\''")
            svc=$(grep -h "reboot_service:" $files 2>/dev/null | tail -1 | awk "{print \$2}" | tr -d "\"'\''")
            [ -n "$topic" ] && [ -n "$svc" ] && echo "$topic $svc" > $cfg
            kill $p
        fi
        cat $cfg 2>/dev/null' 2>/dev/null
}

# Watches the wrist Orbbec and reboots it through its driver while it sends no frames:
# first after WRIST_STALL_S, then every WRIST_RETRY_S, without the growing wait of the
# built-in watchdog. Runs beside the scene camera loop, so neither delays the other.
wrist_loop() {
    local cfg topic svc started age down_since=0
    while true; do
        until container_running; do sleep 2; done
        CUR_INSTANCE=$(container_instance)
        until docker exec "$CONTAINER" grep -q "call)" "$HELPER" 2>/dev/null; do sleep 1; done
        cfg=$(wrist_config)
        topic=${cfg%% *} svc=${cfg##* }
        [ -n "$cfg" ] || topic=/camera/wrist_orbbec/color/camera_info svc=/camera/wrist_orbbec/reboot_device
        log "wrist: watching wrist_orbbec (frames via $topic, reboot via $svc)."
        started=$SECONDS next_try=0 down_since=0
        while same_container; do
            sleep 1
            ensure_frame_watch "$topic" "$WRIST_FILE_CONTAINER" wrist_orbbec_frame_watch
            age=$(file_age "$WRIST_FILE_HOST")
            if ((age <= WRIST_STALL_S)); then
                ((down_since)) && log "wrist: wrist_orbbec is publishing again ($((SECONDS - down_since)) s)."
                down_since=0
                continue
            fi
            # Let a fresh drivers launch open the camera first.
            (($(container_age) >= WRIST_STARTUP_GRACE_S)) || continue
            if ((down_since == 0)); then
                down_since=$SECONDS
                log "wrist: down, no frames for ${age} s."
            fi
            ((SECONDS >= next_try)) || continue
            next_try=$((SECONDS + WRIST_RETRY_S))
            if ! usb_present "$ORBBEC_ID"; then
                log "wrist:   the Orbbec is not on the USB bus; it needs a replug."
                continue
            fi
            log "wrist:   rebooting it through $svc"
            docker exec -u "$USER_NAME" "$CONTAINER" "$HELPER" call "$svc" std_srvs/srv/Empty \
                >/dev/null 2>&1 || log "wrist:   the reboot call failed (is the Orbbec node running?)"
        done
        log "wrist: $CONTAINER stopped; waiting for it."
    done
}

# Make the container's /dev/video* and /dev/media* match the host's, name for name.
mirror_video_nodes() {
    local nodes
    nodes=$(for n in /dev/video* /dev/media*; do [ -e "$n" ] && stat -c "%n %t %T" "$n"; done)
    docker exec -i "$CONTAINER" bash -c '
        want=$(cat)
        for n in /dev/video* /dev/media*; do
            [ -e "$n" ] || continue
            grep -q "^$n " <<<"$want" || rm -f "$n"
        done
        while read -r n maj min; do
            [ -n "$n" ] || continue
            [ "$(stat -c "%t %T" "$n" 2>/dev/null)" = "$maj $min" ] && continue
            rm -f "$n"
            mknod "$n" c $((16#$maj)) $((16#$min)) && chgrp video "$n" && chmod 660 "$n"
        done <<<"$want"' <<<"$nodes"
}

# Save the healthy node's command line and parameter files, once per container lifetime
# (the drivers launch's own node gives exactly the parameters it was started with).
save_node_cmdline() {
    docker exec "$CONTAINER" bash -c "
        [ -s $NODE_CMD ] && exit 0
        p=\$(pgrep -f '$NODE_RE' | head -1); [ -n \"\$p\" ] || exit 1
        mapfile -d '' -t cmd < /proc/\$p/cmdline
        out=() n=0
        for a in \"\${cmd[@]}\"; do
            if [ \"\${prev:-}\" = --params-file ]; then
                n=\$((n + 1)); cp \"\$a\" /tmp/${CAMERA}_params_\$n.yaml || exit 1
                chmod 644 /tmp/${CAMERA}_params_\$n.yaml; a=/tmp/${CAMERA}_params_\$n.yaml
            fi
            out+=(\"\$a\"); prev=\$a
        done
        [ \$n -gt 0 ] || exit 1
        printf '%s\0' \"\${out[@]}\" > $NODE_CMD.tmp && mv $NODE_CMD.tmp $NODE_CMD" 2>/dev/null
}

# The old node holds dead file descriptors, so there is nothing to shut down gracefully.
stop_node() {
    docker exec "$CONTAINER" bash -c "
        pkill -f '[p]ro_rs_launch.py camera_name:=$CAMERA' || true
        pkill -f '$NODE_RE' || true
        for _ in 1 2 3 4; do pgrep -f '$NODE_RE' >/dev/null || exit 0; sleep 0.5; done
        pkill -9 -f '$NODE_RE' || true
        sleep 0.3"
}

# Wait until the D415 and all its video devices are on the bus and nothing about it has
# changed for SETTLE_S. Returns 1 if the container went away meanwhile.
wait_for_settled_d415() {
    local id last="" since=$SECONDS
    while same_container; do
        id=$(d415_usb_id)
        if [ -z "$id" ] || [ "${id##*#}" = 0 ] || [ "$id" != "$last" ]; then
            since=$SECONDS
        elif ((SECONDS - since >= SETTLE_S)); then
            return 0
        fi
        last=$id
        sleep 0.5
    done
    return 1
}

# One recovery attempt. 0: frames are back.
recover_once() {
    local args start id i
    wait_for_settled_d415 || return 1
    id=$(d415_usb_id)
    stop_node
    mirror_video_nodes
    start=$(date +%s)
    if docker exec "$CONTAINER" test -s "$NODE_CMD"; then
        docker exec -d -u "$USER_NAME" "$CONTAINER" "$HELPER" node
    else
        mapfile -t args < <(read_launch_args) || true
        docker exec -d -u "$USER_NAME" "$CONTAINER" "$HELPER" launch "${args[@]}"
    fi
    for ((i = 0; i < FRAME_WAIT_S; i++)); do
        sleep 1
        ensure_frame_watch
        if (($(stat -c %Y "$FRAME_FILE_HOST" 2>/dev/null || echo 0) >= start)); then
            return 0
        fi
        # Renumbered again while its node opened it: this attempt cannot work. The first
        # open after a hub reset nearly always ends this way (a UVC probe error, then a
        # port reset), and the next attempt works, so retry at once.
        if [ "$(d415_usb_id)" != "$id" ] \
            || docker exec "$CONTAINER" grep -q "No such device or address" "$LOG" 2>/dev/null; then
            log "  the D415 was renumbered while opening; retrying"
            return 2
        fi
    done
    docker exec "$CONTAINER" grep -vE '^\s*$' "$LOG" 2>/dev/null | tail -n 3 | sed 's/^/    /'
    return 1
}

recover() {
    local attempt=1 backoff=1 t0=$SECONDS
    while same_container; do
        log "Recovery attempt $attempt..."
        recover_once
        rc=$?
        if [ $rc = 0 ]; then
            log "scene_camera is publishing again (attempt $attempt, $((SECONDS - t0)) s)."
            return 0
        fi
        attempt=$((attempt + 1))
        [ $rc = 2 ] && continue  # renumbered while opening: the expected path, no wait
        sleep "$backoff"
        backoff=$((backoff * 2 > 20 ? 20 : backoff * 2))
    done
    return 1
}

if ! read_launch_args >/dev/null; then
    echo "Cannot read the $CAMERA include from $CAMERAS_LAUNCH." >&2
    exit 1
fi

wrist_loop &
trap 'kill $(jobs -p) 2>/dev/null' EXIT

log "Watching $CAMERA (frames via $INFO_TOPIC, D415 $D415_ID)."
while true; do
    until container_running; do sleep 2; done
    CUR_INSTANCE=$(container_instance)
    install_helpers
    ensure_frame_watch
    # Give the drivers launch time to open the camera, but start watching as soon as it does.
    while same_container && (($(frame_age) > STALL_S)) && (($(container_age) < STARTUP_GRACE_S)); do
        sleep 1
        ensure_frame_watch
    done
    usb_id=$(d415_usb_id)
    reason=""
    while same_container; do
        sleep "$POLL_S"
        (($(frame_age) <= STALL_S)) && save_node_cmdline
        now_id=$(d415_usb_id)
        age=$(frame_age)
        if [ "$now_id" != "$usb_id" ]; then
            reason="the D415 re-enumerated (${usb_id:-absent} -> ${now_id:-absent})"
        elif ((age > STALL_S)); then
            # A dead frame watcher looks the same as a dead camera; rule it out first.
            if ! frame_watch_running "$INFO_TOPIC"; then
                ensure_frame_watch
                sleep 3
                (($(frame_age) > STALL_S)) || continue
            fi
            reason="no frames for ${age} s"
        fi
        if [ -n "$reason" ] && same_container; then
            log "Down: $reason."
            recover
            usb_id=$(d415_usb_id)
            reason=""
        fi
    done
    log "$CONTAINER stopped; waiting for it."
done
