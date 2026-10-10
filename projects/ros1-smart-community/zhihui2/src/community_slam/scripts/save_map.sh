#!/usr/bin/env bash
set -euo pipefail
output="${1:-$(rospack find community_slam)/maps/smart_community}"
rosrun map_server map_saver -f "$output"
