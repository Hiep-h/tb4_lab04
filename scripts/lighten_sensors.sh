#!/usr/bin/env bash
# Giam tai mo phong: ha tan so/do phan giai LiDAR va camera OAK-D cua TurtleBot 4 trong goi turtlebot4_description.
# Mac dinh LiDAR 62 Hz x 640 tia + camera RGB-D 30 Hz => dung hinh bang phan mem cuc cham (Real Time Factor ~ 1%).
# Sau khi chay: LiDAR 10 Hz x 360 tia, camera 2 Hz. Can sudo. Go bo: scripts/lighten_sensors.sh restore
set -e
source /opt/ros/humble/setup.bash
D="$(ros2 pkg prefix turtlebot4_description)/share/turtlebot4_description/urdf/sensors"
LIDAR="$D/rplidar.urdf.xacro"; CAM="$D/oakd.urdf.xacro"
if [ "$1" = "restore" ]; then
  for f in "$LIDAR" "$CAM"; do [ -f "$f.orig" ] && sudo cp "$f.orig" "$f"; done
  echo "Da khoi phuc cau hinh goc"; exit 0
fi
for f in "$LIDAR" "$CAM"; do [ -f "$f.orig" ] || sudo cp "$f" "$f.orig"; done
sudo sed -i -e 's/update_rate="62.0"/update_rate="10.0"/' -e 's/h_samples="640"/h_samples="360"/' -e 's/visualize="1"/visualize="0"/' "$LIDAR"
sudo sed -i -e 's#<update_rate>30</update_rate>#<update_rate>2</update_rate>#' -e 's#<visualize>true</visualize>#<visualize>false</visualize>#' "$CAM"
echo "--- LiDAR:"; grep -n 'update_rate\|h_samples\|visualize' "$LIDAR"
echo "--- Camera:"; grep -n 'update_rate\|visualize' "$CAM"
