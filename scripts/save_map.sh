#!/usr/bin/env bash
# Phan 2: luu ban do (map.yaml + map.pgm) vao thu muc maps/ cua repo.
# Dung: scripts/save_map.sh [thu_muc_dich]   (mac dinh: ../maps)
set -e
source /opt/ros/humble/setup.bash
OUT="${1:-$(cd "$(dirname "$0")/.." && pwd)/maps}"
mkdir -p "$OUT"
ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p use_sim_time:=true -p save_map_timeout:=30.0 \
  || ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$OUT/map'}}"
ls -l "$OUT"/map.yaml "$OUT"/map.pgm
