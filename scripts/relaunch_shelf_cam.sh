#!/usr/bin/env bash
# Relaunch the shelf_cam ZED 2i inside the running drivers container, without restarting
# the stack, then wait for its colour frames. Run from the host.
#
# Stops the three shelf_cam processes the drivers launch started (the ZED component
# container, the RGB relay and the camera's state publisher) and starts pro_zed_launch.py
# again with the camera's serial and model read from cameras.launch.xml. The relaunched
# processes belong to this script's launch, not the drivers launch: a drivers restart
# replaces them with the normal ones.
#
# It first checks whether the shelf_cam is already publishing (then it does nothing) and
# whether the ZED node is still starting up (then it waits for it), because killing a ZED
# that is opening the camera leaves the camera unopenable for a while.
#
# A relaunch fixes a ZED node that crashed or gave up opening the camera. It cannot fix a
# camera the container does not see: the drivers container's /dev is a snapshot from when
# it started, so after the ZED drops off the USB bus and re-enumerates, only a drivers
# restart lets the container see it again. The script checks for that and says so.
#
# CALIBRATION_MODE=1 uses the calibration resolution parameters. WAIT_S sets how long to
# wait for frames. See design_docs/zed_shelf_cam.md.
set -euo pipefail

CONTAINER="${CONTAINER:-moveit_pro-drivers-1}"
WAIT_S="${WAIT_S:-45}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERAS_LAUNCH="$WS/src/picknik_009_ur5e_hw_config/launch/cameras.launch.xml"
PARAMS_FILE=zed_shelf_cam.yaml
[ "${CALIBRATION_MODE:-0}" = "1" ] && PARAMS_FILE=zed_shelf_cam_calibration.yaml

# Read the shelf_cam include's args from cameras.launch.xml so this never drifts from it.
read -r SERIAL MODEL < <(python3 - "$CAMERAS_LAUNCH" <<'EOF'
import sys, xml.etree.ElementTree as ET
for inc in ET.parse(sys.argv[1]).getroot().iter("include"):
    args = {a.get("name"): a.get("value") for a in inc.findall("arg")}
    if args.get("camera_name") == "shelf_cam":
        print(args["serial_number"], args.get("camera_model", "zed2i"))
        break
else:
    sys.exit("no shelf_cam include in " + sys.argv[1])
EOF
)
echo "shelf_cam: ZED $MODEL, serial $SERIAL, params $PARAMS_FILE"

docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true \
    || { echo "$CONTAINER is not running; start the stack first." >&2; exit 1; }

# Is the ZED on the host bus, and does the container have that device node? A node the
# container lacks means the camera re-enumerated after the container started.
HOST_DEV=$(lsusb -d 2b03:f880 2>/dev/null | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
if [ -z "$HOST_DEV" ]; then
    echo "The host does not see the ZED (USB 2b03:f880). Check its cable and power, replug it," >&2
    echo "then restart the drivers container (docker restart $CONTAINER)." >&2
    exit 1
fi
if ! docker exec "$CONTAINER" test -e "$HOST_DEV"; then
    echo "The ZED is on the host at $HOST_DEV, but the drivers container does not have that node:" >&2
    echo "it re-enumerated after the container started. Restart the drivers container" >&2
    echo "(docker restart $CONTAINER); a relaunch cannot reach it." >&2
    exit 1
fi

# True when a colour frame arrives within 4 s.
shelf_cam_publishing() {
    docker exec -u "$(id -un)" "$CONTAINER" bash -c "
        source ~/.moveit-pro-bashrc >/dev/null 2>&1
        export \$(tr '\0' '\n' < /proc/\$(pgrep -f ros2_control_node | head -1)/environ \
            | grep -E '^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)' | grep -v '=\$' | xargs)
        timeout 4 ros2 topic echo --once --no-arr /camera/shelf_cam/color/image_raw sensor_msgs/msg/Image" \
        >/dev/null 2>&1
}

# Never kill a ZED that works or is still coming up. The ZED SDK takes several seconds to
# open the camera after a stack start, and killing it mid-open (or right after) leaves the
# camera unopenable for a while: that is how a good restart once turned into a dead camera.
if shelf_cam_publishing; then
    echo "shelf_cam is already publishing; nothing to do."
    exit 0
fi
ZED_AGE=$(docker exec "$CONTAINER" bash -c 'p=$(pgrep -f "[_]_ns:=/shelf_cam.*zed_container|zed_container.*[_]_ns:=/shelf_cam" | head -1); [ -n "$p" ] && ps -o etimes= -p "$p" | tr -d " "' || true)
if [ -n "$ZED_AGE" ] && [ "$ZED_AGE" -lt 60 ]; then
    echo "The ZED node started ${ZED_AGE}s ago; waiting for it rather than restarting it..."
    for ((i = ZED_AGE; i < 60; i += 5)); do
        sleep 5
        if shelf_cam_publishing; then echo "shelf_cam is publishing."; exit 0; fi
    done
    echo "Still no frames after a minute; relaunching it."
fi

echo "Stopping the running shelf_cam processes..."
# Bracketed patterns ([p]ro_zed...) so pkill does not match this bash -c command itself.
docker exec "$CONTAINER" bash -c '
    pkill -f "[p]ro_zed_launch.py" || true
    pkill -f "[_]_ns:=/shelf_cam" || true
    pkill -f "[s]helf_cam_rgb_image_relay" || true
    for _ in $(seq 20); do pgrep -f "[_]_ns:=/shelf_cam|[s]helf_cam_rgb_image_relay" >/dev/null || exit 0; sleep 0.5; done
    pkill -9 -f "[_]_ns:=/shelf_cam|[s]helf_cam_rgb_image_relay" || true'
# The SDK can still hold the camera for a moment after its process is gone.
sleep 5

echo "Relaunching (log: /tmp/shelf_cam_relaunch.log in $CONTAINER)..."
# -u by name, not uid:gid: docker applies the user's supplementary groups only for a name,
# and without the video group the ZED SDK cannot open /dev/video* and reports that as
# "CAMERA NOT DETECTED".
# Same DDS settings as the drivers launch, so the relaunched nodes join the same graph.
docker exec -d -u "$(id -un)" "$CONTAINER" bash -c "
    source ~/.moveit-pro-bashrc >/dev/null 2>&1
    export \$(tr '\0' '\n' < /proc/\$(pgrep -f ros2_control_node | head -1)/environ \
        | grep -E '^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)' | grep -v '=\$' | xargs)
    exec ros2 launch picknik_009_ur5e_hw_config pro_zed_launch.py \
        camera_name:=shelf_cam camera_model:=$MODEL serial_number:=$SERIAL \
        ros_params_override_path:=\$(ros2 pkg prefix picknik_009_ur5e_hw_config)/share/picknik_009_ur5e_hw_config/config/$PARAMS_FILE \
        > /tmp/shelf_cam_relaunch.log 2>&1"

echo "Waiting up to ${WAIT_S}s for frames on /camera/shelf_cam/color/image_raw..."
for ((i = 0; i < WAIT_S; i += 5)); do
    sleep 5
    if docker exec "$CONTAINER" grep -q "CAMERA NOT DETECTED" /tmp/shelf_cam_relaunch.log 2>/dev/null \
        && ! docker exec "$CONTAINER" pgrep -f "[_]_ns:=/shelf_cam" >/dev/null 2>&1; then
        break
    fi
    if shelf_cam_publishing; then
        echo "shelf_cam is publishing again."
        exit 0
    fi
done

echo "No shelf_cam frames. Last lines of the relaunch log:" >&2
docker exec "$CONTAINER" tail -n 15 /tmp/shelf_cam_relaunch.log >&2 || true
if docker exec "$CONTAINER" grep -q "CAMERA NOT DETECTED" /tmp/shelf_cam_relaunch.log 2>/dev/null; then
    echo "The ZED SDK cannot open the camera although the container has its device node." >&2
    echo "The camera is likely wedged after a USB drop: unplug it, wait a few seconds, plug it" >&2
    echo "back in, then restart the drivers container (the replug gives it new device nodes)." >&2
fi
exit 1
