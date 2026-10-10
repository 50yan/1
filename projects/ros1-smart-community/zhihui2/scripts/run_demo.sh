#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace/devel/setup.bash"
roslaunch community_gazebo simulation.launch &
gazebo_pid=$!
trap 'kill "$gazebo_pid" 2>/dev/null || true' EXIT
sleep 5
roslaunch community_slam localization.launch &
localization_pid=$!
trap 'kill "$localization_pid" 2>/dev/null || true' EXIT
sleep 2
roslaunch community_navigation navigation.launch use_rviz:=false &
navigation_pid=$!
trap 'kill "$navigation_pid" 2>/dev/null || true' EXIT
sleep 2
roslaunch community_mission mission.launch
