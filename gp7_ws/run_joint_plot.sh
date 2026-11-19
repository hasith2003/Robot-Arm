#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=91
exec python3 plot_joint_variables.py "$@"
