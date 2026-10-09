#!/usr/bin/env bash
# Cai world "small_house" (nha o AWS RoboMaker ban nhe) vao goi turtlebot4_ignition_bringup de dung world:=small_house.
# Can sudo vi ghi vao /opt/ros/humble/share/... Dung: scripts/install_world.sh   (go bo: scripts/install_world.sh remove)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
DEST="$(ros2 pkg prefix turtlebot4_ignition_bringup)/share/turtlebot4_ignition_bringup/worlds"
if [ "$1" = "remove" ]; then
  sudo rm -rf "$DEST/small_house.sdf" "$DEST"/aws_robomaker_residential_*
  echo "Da go small_house khoi $DEST"; exit 0
fi
sudo cp "$HERE/worlds/small_house.sdf" "$DEST/"
sudo cp -r "$HERE"/models/aws_robomaker_residential_* "$DEST/"
echo "Da cai small_house vao $DEST"
ls "$DEST"
