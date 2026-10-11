#!/bin/bash
set -e
task_backup_dir="$(cd "$(dirname "$0")" && pwd)"
cp "$task_backup_dir/15.world" /home/ros1/ce/src/ceshi3/worlds/15.world
cp "$task_backup_dir/CMakeLists.txt" /home/ros1/ce/src/ceshi3/CMakeLists.txt
cp "$task_backup_dir/package.xml" /home/ros1/ce/src/ceshi3/package.xml
echo 'Original traffic-light animation restored; restart robot.launch.'
