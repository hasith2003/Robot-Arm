#!/usr/bin/env bash
# Keep this test away from leftover simulators and controller managers.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=89
export GZ_PARTITION=gp7_verified_motion_89
exec ros2 launch gp7_simulation simulation.launch.py demo:=true "$@"
