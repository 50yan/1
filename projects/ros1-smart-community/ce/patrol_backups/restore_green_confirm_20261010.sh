#!/bin/bash
set -e
cp -a "/home/ros1/ce/patrol_backups/before_timed_traffic_20261010_201448/patrol_with_vision.py" /home/ros1/ce/src/ceshi3/scripts/patrol_with_vision.py
cp -a "/home/ros1/ce/patrol_backups/before_timed_traffic_20261010_201448/package.xml" /home/ros1/ce/src/ceshi3/package.xml
source /opt/ros/noetic/setup.bash
cd /home/ros1/ce
python3 -m py_compile src/ceshi3/scripts/patrol_with_vision.py
catkin_make --pkg ceshi3
echo "Restored the previous green-confirmation version. Restart patrol_with_vision.launch."
