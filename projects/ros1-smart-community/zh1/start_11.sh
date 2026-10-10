#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/ros1/zh1/devel/setup.bash

# VMware's virtual GPU can crash Gazebo Classic's default renderer.
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export LIBGL_ALWAYS_SOFTWARE=1
export SVGA_VGPU10=0
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$LD_LIBRARY_PATH

if ! rostopic list >/dev/null 2>&1; then
  roscore >/tmp/zh1-roscore.log 2>&1 &
  sleep 3
fi

exec roslaunch zhihui_shequ_sim run_11.launch "$@"
