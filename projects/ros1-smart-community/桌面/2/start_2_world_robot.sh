#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /home/ros1/catkin_ws2/devel_isolated/setup.bash
set -u

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

if pgrep -u ros1 -f 'roslaunch map_build load_robot_in_2_world.launch' >/dev/null; then
  echo '2.world and TurtleBot3 are already running.'
  exit 0
fi

exec roslaunch map_build load_robot_in_2_world.launch "$@"
