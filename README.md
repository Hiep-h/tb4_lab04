# Bài thực hành 04 – TurtleBot 4 Simulation (SLAM Toolbox + Nav2)

TurtleBot 4 trong Gazebo (Ignition Fortress, ROS 2 Humble): xây dựng bản đồ bằng SLAM Toolbox, lưu bản đồ, localization (AMCL) và điều hướng tự động bằng Nav2 trên bản đồ đã lưu.

- Sinh viên: Vũ Văn Hiệp – MSSV 230200742
- Package `tb4_lab04`: launch file bọc các launch có sẵn của TurtleBot 4, bảng điều khiển thanh trượt, node tự khám phá, world `small_house` và bản đồ. Không tự xây lại simulator hay controller.
- Video demo: https://drive.google.com/drive/folders/12Sag4V-dFMzuR-fBfql6GcY7HkK_qXRz?usp=sharing

## 1. Cấu trúc repo

```
tb4_lab04/
├── launch/
│   ├── sim.launch.py            TurtleBot 4 trong Gazebo
│   ├── mapping.launch.py        SLAM Toolbox + RViz2
│   ├── localization.launch.py   nạp map + AMCL + RViz2
│   ├── navigation.launch.py     Nav2
│   ├── nav_demo.launch.py       Gazebo + AMCL + Nav2 + RViz2 (một lệnh, dùng map đã lưu)
│   ├── drive_mapping.launch.py  Gazebo + SLAM + RViz2 + bảng điều khiển thanh trượt
│   └── auto_mapping.launch.py   Gazebo + SLAM + RViz2 + node tự khám phá
├── tb4_lab04/
│   ├── control_panel.py         bảng điều khiển thanh trượt (Tkinter), lưu bản đồ
│   ├── auto_explore.py          node tự khám phá theo biên (frontier)
│   ├── explorer.py, frontier.py thuật toán khám phá (thuần Python)
│   ├── wall_follow.py           tính lệnh bám tường từ LiDAR
│   └── map_saver.py             lưu bản đồ bằng map_saver_cli
├── maps/                        map.yaml + map.pgm (bản gốc SLAM), map_clean.* (đã làm sạch)
├── worlds/, models/             world small_house (AWS RoboMaker, MIT-0) bản nhẹ
├── scripts/                     check_topics.sh, save_map.sh, install_world.sh, lighten_sensors.sh
├── tools/clean_map.py           làm sạch thân đồ nội thất trong map.pgm
└── package.xml, setup.py, setup.cfg
```

## 2. Cài đặt

Ubuntu 22.04, ROS 2 Humble Desktop.

```bash
sudo apt update
sudo apt install -y python3-tk ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard

mkdir -p ~/tb4_ws/src && cd ~/tb4_ws/src
git clone https://github.com/Hiep-h/tb4_lab04.git
cd ~/tb4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_lab04 --symlink-install
source install/setup.bash
```

Mỗi terminal mới: `source /opt/ros/humble/setup.bash && source ~/tb4_ws/install/setup.bash`.

World `small_house` cần cài vào gói `turtlebot4_ignition_bringup` một lần (cần `sudo`):

```bash
~/tb4_ws/src/tb4_lab04/scripts/install_world.sh
```

Gazebo dựng bằng phần mềm trên card Intel nên chậm. Giảm tải cảm biến (LiDAR 10 Hz × 360 tia, camera 2 Hz) một lần:

```bash
~/tb4_ws/src/tb4_lab04/scripts/lighten_sensors.sh
```

## 3. Phần 1 – TurtleBot 4 trong Gazebo

```bash
ros2 launch tb4_lab04 sim.launch.py world:=small_house x:=1.0 y:=1.0 rviz:=true
./scripts/check_topics.sh
```

`world`: `small_house`, `small_house_lite`, `maze`, `warehouse`, `depot`. Kiểm tra `/scan`, `/odom` có dữ liệu và robot nhận được `/cmd_vel`:

```bash
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 4. Phần 2 – Xây dựng bản đồ bằng SLAM Toolbox

**Lái bằng bảng thanh trượt (teleoperation):**

```bash
ros2 launch tb4_lab04 drive_mapping.launch.py world:=small_house x:=1.0 y:=1.0
```

Mở Gazebo, SLAM Toolbox, RViz2 và bảng điều khiển. Kéo thanh **Tiến/Lùi** sang phải để tiến, thanh **Xoay tròn** để quay, SPACE để phanh. Nút *LƯU BẢN ĐỒ NGAY* ghi `maps/map.yaml` và `maps/map.pgm`; bản đồ cũng tự lưu định kỳ và khi đóng bảng. Launch đặt `safety_override:=full` cho `/motion_control` (Create 3 giới hạn việc lùi).

**Robot tự khám phá:**

```bash
ros2 launch tb4_lab04 auto_mapping.launch.py world:=small_house x:=1.0 y:=1.0
```

Node `auto_explore` chọn vùng chưa biết gần nhất theo đường đi thật (Dijkstra trên bản đồ đã phồng quanh vật cản), bám đường bằng pure pursuit và tự lưu bản đồ khi hết vùng chưa biết.

**Lưu thủ công:**

```bash
./scripts/save_map.sh
```

`maps/map.yaml` + `maps/map.pgm` là bản đồ gốc do SLAM Toolbox sinh ra. `maps/map_clean.*` là cùng bản đồ sau khi `tools/clean_map.py` điền thân đồ nội thất (ô chưa biết nằm trong chu vi nhà) thành vật cản.

## 5. Phần 3 và 4 – Localization và Nav2

```bash
ros2 launch tb4_lab04 nav_demo.launch.py world:=small_house x:=1.0 y:=1.0
```

Launch chờ Gazebo và robot sẵn sàng, rồi chạy AMCL với `maps/map_clean.yaml` (thêm `map:=$HOME/tb4_ws/src/tb4_lab04/maps/map.yaml` để dùng bản gốc), đặt vị trí ban đầu (0, 0, 0) và bật Nav2. Trong RViz2 dùng **Nav2 Goal** (bấm và kéo để chọn hướng) đặt ít nhất 3 điểm đích ở các khu vực khác nhau, không điều khiển thủ công.

Chạy từng bước:

```bash
ros2 launch tb4_lab04 sim.launch.py world:=small_house x:=1.0 y:=1.0
ros2 launch tb4_lab04 localization.launch.py map:=$HOME/tb4_ws/src/tb4_lab04/maps/map_clean.yaml
ros2 launch tb4_lab04 navigation.launch.py
```

## 6. Xử lý sự cố

| Hiện tượng | Cách xử lý |
|---|---|
| `/scan` toàn 0.0, bản đồ rỗng | LiDAR GPU lỗi trên một số card Intel. Các launch bật `LIBGL_ALWAYS_SOFTWARE=1` mặc định cho Gazebo (`software_render:=false` để tắt) |
| Gazebo chậm, Real Time Factor thấp | Chạy `scripts/lighten_sensors.sh`, thu nhỏ cửa sổ Gazebo, dùng `world:=small_house_lite` |
| Chạy lại bị trùng tiến trình | `pkill -9 -x ruby; pkill -9 -f "ros2 launch"` rồi chạy lại |
| Robot không bám map | Dùng **2D Pose Estimate** trong RViz2 |
| Nav2 báo goal thất bại | Đặt goal vào vùng trống, cách vật cản xa |
