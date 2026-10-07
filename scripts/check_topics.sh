#!/usr/bin/env bash
# Phan 1: kiem tra /scan, /odom, /cmd_vel (chay khi mo phong dang chay).
set -u
source /opt/ros/humble/setup.bash
echo "== Danh sach topic =="; ros2 topic list
for t in /scan /odom; do
  echo; echo "== $t =="; ros2 topic info "$t"; ros2 topic hz "$t" --window 20 &
  pid=$!; sleep 5; kill $pid 2>/dev/null; wait $pid 2>/dev/null
done
echo; echo "== /cmd_vel =="; ros2 topic info /cmd_vel
echo; echo "Thu gui lenh van toc (robot tien 2 giay roi dung):"
timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" >/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null
echo "Xong."
