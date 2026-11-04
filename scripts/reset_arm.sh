#!/usr/bin/env bash
# Run reset_arm.py inside the running drivers container from the host: the UI's Reset Arm,
# which the Desktop App cannot call. Arguments are passed through (e.g. --force).
set -euo pipefail

CONTAINER="${CONTAINER:-moveit_pro-drivers-1}"

exec docker exec -i -u "$(id -u):$(id -g)" "$CONTAINER" \
    bash -c 'source ~/.moveit-pro-bashrc >/dev/null 2>&1; python3 -u ~/user_ws/scripts/reset_arm.py "$@"' \
    reset_arm "$@"
