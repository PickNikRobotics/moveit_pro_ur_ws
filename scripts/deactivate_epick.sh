#!/usr/bin/env bash
# Run deactivate_epick.py inside the running dev-jetson container from the host.
set -euo pipefail

MOVEIT_PRO_DIR="${MOVEIT_PRO_DIR:-/mnt/nvme/code/moveit_pro}"
SERVICE="${SERVICE:-dev-jetson}"

cd "$MOVEIT_PRO_DIR"

for i in 2 1; do
    echo "Deactivating ePick in ${i}..."
    sleep 1
done

exec docker compose --env-file .env exec -T -u picknikuser "$SERVICE" \
    bash -lc 'source ~/.moveit-pro-bashrc && python3 ~/user_ws/scripts/deactivate_epick.py'
