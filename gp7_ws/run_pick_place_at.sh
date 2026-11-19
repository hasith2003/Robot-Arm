#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ $# -lt 2 ]]; then
    echo 'Usage: bash run_pick_place_at.sh X_metres Y_metres [launch arguments]' >&2
    exit 2
fi
pickup_x=$1
pickup_y=$2
shift 2
pickup_world=$(mktemp /tmp/gp7-pickup-XXXXXX.sdf)
trap 'rm -f "$pickup_world"' EXIT
python3 box_position_world.py "$pickup_x" "$pickup_y" "$pickup_world"
bash run_pick_place.sh "world_file:=$pickup_world" "$@"
