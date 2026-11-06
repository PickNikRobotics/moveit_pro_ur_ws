#!/usr/bin/env bash
# Watch the wrist_mounted_camera D415, say what is wrong the moment it stops, and relaunch
# it when a relaunch can help. Run from the host; long-running, Ctrl-C stops it.
#
# Runs the shelf_stocking camera_watchdog.py on the wrist colour topic inside the drivers
# container and prints its LIVE / SLOW / STALLED transitions. Once the camera has been
# STALLED for GRACE_S it runs relaunch_wrist_camera.sh --check (off the USB bus? missing
# from the container's /dev? or just a dead node?) and prints the diagnosis, again every
# RECHECK_S while it stays down if the diagnosis changes. When only the node is dead it
# runs relaunch_wrist_camera.sh, once per outage. It never restarts the drivers container
# or USB-resets the camera: both take an operator (a drivers restart drops arm control).
#
# It survives stack restarts: when the drivers container goes away it waits for it and
# starts watching again.
#
# AUTO_RELAUNCH=0 reports only. NOTIFY=0 turns off the desktop notification.
# See design_docs/wrist_camera_recovery.md.
set -uo pipefail

CONTAINER="${CONTAINER:-moveit_pro-drivers-1}"
GRACE_S="${GRACE_S:-10}"
RECHECK_S="${RECHECK_S:-60}"
SUMMARY_S="${SUMMARY_S:-300}"
AUTO_RELAUNCH="${AUTO_RELAUNCH:-1}"
NOTIFY="${NOTIFY:-1}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELAUNCH="$HERE/relaunch_wrist_camera.sh"
TOPIC=/camera/wrist_mounted_camera/color/image_raw
WATCHDOG=~/user_ws/src/shelf_stocking/scripts/camera_watchdog.py

log() { echo "[$(date +%H:%M:%S)] $*"; }
notify() {
    [ "$NOTIFY" = 1 ] && command -v notify-send >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] \
        && notify-send -u critical "Wrist camera" "$1" 2>/dev/null || true
}

container_running() {
    docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true
}

# The watchdog runs as a coprocess whose stdin is a pipe this shell holds open. When this
# shell exits, by any means, the pipe closes, the cat below sees EOF and stops the watchdog:
# docker exec does not stop the process in the container when its client dies. cat reads
# fd 3 because a background command's stdin is otherwise /dev/null. The watchdog exiting by
# itself ends the coprocess, so a dead watchdog is noticed rather than silently waited on.
start_watchdog() {
    # shellcheck disable=SC2016
    coproc WATCH {
        exec docker exec -i -u "$(id -un)" "$CONTAINER" bash -c '
            source ~/.moveit-pro-bashrc >/dev/null 2>&1
            export $(tr "\0" "\n" < /proc/$(pgrep -f ros2_control_node | head -1)/environ \
                | grep -E "^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)" | grep -v "=\$" | xargs)
            exec 3<&0
            python3 -u "$0" "$@" 2>&1 & w=$!
            ( cat <&3 >/dev/null; kill $w 2>/dev/null ) &
            wait $w' "$WATCHDOG" "$TOPIC" --summary "$SUMMARY_S" 2>&1
    }
}

log "Watching $TOPIC (auto relaunch: $([ "$AUTO_RELAUNCH" = 1 ] && echo on || echo off))."
while true; do
    if ! container_running; then
        log "$CONTAINER is not running; waiting for it..."
        until container_running; do sleep 5; done
        # Let the drivers launch bring the cameras up before judging them.
        sleep 20
    fi
    start_watchdog
    # Our own copies of the coprocess pipes: bash drops WATCH once the coprocess exits, and
    # holding the write end is what keeps the watchdog's stdin open.
    exec {out}<&"${WATCH[0]}" {in}>&"${WATCH[1]}"
    pid=$WATCH_PID
    state="" stalled_since=0 last_check=0 last_diag="" relaunched=0
    while true; do
        IFS= read -r -t 5 -u "$out" line
        rc=$?
        if [ $rc = 0 ]; then
            line="${line#\[*\] }"  # the watchdog's own (container, UTC) timestamp
            case "$line" in
                STALLED*)
                    log "$line"
                    notify "Stalled: no frames on $TOPIC"
                    state=STALLED stalled_since=$SECONDS last_check=0 last_diag=""
                    ;;
                LIVE*|SLOW*)
                    log "$line"
                    [ "$state" = STALLED ] && notify "Back: ${line%% *}"
                    state=${line%% *} relaunched=0
                    ;;
                *) log "$line" ;;
            esac
        elif [ $rc -le 128 ]; then
            break  # EOF: the watchdog or its container went away
        fi

        [ "$state" = STALLED ] || continue
        ((SECONDS - stalled_since >= GRACE_S)) || continue
        ((last_check == 0 || SECONDS - last_check >= RECHECK_S)) || continue
        last_check=$SECONDS
        diag=$("$RELAUNCH" --check 2>&1)
        code=$?
        if [ "$diag" != "$last_diag" ]; then
            sed 's/^/    /' <<<"$diag"
            last_diag=$diag
        fi
        if [ "$code" = 4 ] && [ "$AUTO_RELAUNCH" = 1 ] && [ "$relaunched" = 0 ]; then
            relaunched=1
            log "Relaunching the wrist camera node..."
            "$RELAUNCH" 2>&1 | sed 's/^/    /'
        fi
    done
    exec {out}<&- {in}>&-
    wait "$pid" 2>/dev/null
    log "The watchdog stopped; restarting it in 5 s."
    sleep 5
done
