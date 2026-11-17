#!/usr/bin/env bash
# Keep this test away from leftover simulators and controller managers.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=89
export GZ_PARTITION=gp7_verified_motion_89
# Own a process group so Gazebo children cannot survive a stopped launch.
# The shared lock also prevents the two workspace copies using this domain together.
exec 9>"/tmp/gp7-simulation-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo "A GP7 simulation is already running in ROS domain ${ROS_DOMAIN_ID}." >&2; exit 1; }
simulation_pid=
cleanup() {
    trap '' INT TERM
    if [[ -n "$simulation_pid" ]]; then
        kill -INT -- "-$simulation_pid" 2>/dev/null || true
        sleep 2
        kill -TERM -- "-$simulation_pid" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$simulation_pid" 2>/dev/null || true
        wait "$simulation_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch gp7_simulation simulation.launch.py demo:=true "$@" &
simulation_pid=$!
wait "$simulation_pid"
