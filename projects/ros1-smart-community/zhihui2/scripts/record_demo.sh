#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/devel/setup.bash"
for topic in /scan /odom /camera/image_raw /perception/result /mission/status; do
  rostopic info "$topic" >/dev/null || { echo "Missing required topic: $topic" >&2; exit 1; }
done
exec rosbag record -O "${1:-smart_community_demo}" /scan /odom /camera/image_raw /perception/annotated_image /perception/result /mission/status /tf /tf_static
