#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -f "$HOME/ws/devel/setup.bash" ]; then
  source "$HOME/ws/devel/setup.bash"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.131.1:11311}"
export ROS_IP="${ROS_IP:-192.168.131.100}"
unset ROS_HOSTNAME

exec "$@"
