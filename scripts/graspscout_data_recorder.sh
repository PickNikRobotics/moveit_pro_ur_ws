#!/usr/bin/env bash
# Run graspscout_data_recorder.py inside the running Runtime container from the host: it records a
# GraspScout fine-tuning sample for every pick Unshelve Product Loop plans (Collect Unshelve Episodes
# under the 2F-85). Long-running; Ctrl-C stops it. Restart it after every stack restart, since the
# container it runs in goes away. Arguments are passed through (e.g. --out DIR).
# Samples land in ~/.local/share/moveit_pro/graspscout_data (the same path on the host); export them
# with scripts/export_graspscout_data.py.
set -euo pipefail

CONTAINER="${CONTAINER:-moveit_pro-runtime-1}"

# -u by name so the files are the host user's; the DDS settings of the running objective server so
# the node joins the same graph.
exec docker exec -i -u "$(id -un)" "$CONTAINER" bash -c '
    source ~/.moveit-pro-bashrc >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash
    export $(tr "\0" "\n" < /proc/$(pgrep -f objective_server_node | head -1)/environ \
        | grep -E "^(CYCLONEDDS_|RMW_|ROS_DOMAIN|ROS_AUTOMATIC)" | grep -v "=$" | xargs)
    exec python3 -u ~/user_ws/scripts/graspscout_data_recorder.py "$@"' \
    graspscout_data_recorder "$@"
