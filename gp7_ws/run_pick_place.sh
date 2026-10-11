#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=91
export GZ_PARTITION=gp7_pick_place_91
exec ros2 launch gp7_simulation simulation.launch.py pick_place:=true "$@"
