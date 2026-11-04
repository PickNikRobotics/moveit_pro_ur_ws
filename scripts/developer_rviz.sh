#!/usr/bin/env bash
# Launch developer_rviz inside the running dev-jetson container from the host.
set -euo pipefail

MOVEIT_PRO_DIR="${MOVEIT_PRO_DIR:-/mnt/nvme/code/moveit_pro}"
SERVICE="${SERVICE:-dev-jetson}"

cd "$MOVEIT_PRO_DIR"
exec docker compose --env-file .env exec -u picknikuser "$SERVICE" \
    bash -lc 'source ~/.moveit-pro-bashrc && ros2 launch moveit_studio_agent developer_rviz.launch.py'
