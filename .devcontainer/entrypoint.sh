#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source "/ros2_ws/install/setup.bash"
source "$HOME/.bashrc"
exec $@