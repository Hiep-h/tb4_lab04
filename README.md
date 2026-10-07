# Bài thực hành 04 – TurtleBot 4 Simulation (SLAM Toolbox + Nav2)

Mô phỏng TurtleBot 4 trong Gazebo (Ignition Fortress) trên ROS 2 Humble, xây dựng bản đồ bằng SLAM Toolbox, lưu bản đồ, localization và điều hướng tự động bằng Nav2 trên bản đồ đã lưu.

- Sinh viên: Vũ Văn Hiệp – MSSV 230200742
- Package: `tb4_lab04` (chỉ chứa launch file bọc các launch có sẵn của TurtleBot 4, script hỗ trợ và bản đồ; không tự xây lại simulator hay controller)

## 1. Cấu trúc repo

```
tb4_lab04/
├── launch/
│   ├── sim.launch.py            # Phần 1: TurtleBot 4 trong Gazebo
│   ├── mapping.launch.py        # Phần 2: SLAM Toolbox + RViz2
│   ├── localization.launch.py   # Phần 3: nạp map.yaml + localization + RViz2
│   └── navigation.launch.py     # Phần 4: Nav2
├── maps/                        # map.yaml + map.pgm (sinh ở Phần 2)
├── config/                      # cấu hình tuỳ chỉnh (nếu có)
├── scripts/
│   ├── check_topics.sh          # kiểm tra /scan, /odom, /cmd_vel
│   └── save_map.sh              # lưu bản đồ vào maps/
├── tb4_lab04/                   # python package (rỗng)
├── package.xml, setup.py, setup.cfg
└── README.md
```

## 2. Cài đặt

Yêu cầu: Ubuntu 22.04 (hoặc WSL2 + Ubuntu 22.04), ROS 2 Humble Desktop.

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard
```

Build package này:

```bash
mkdir -p ~/tb4_ws/src && cd ~/tb4_ws/src
git clone https://github.com/Hiep-h/tb4_lab04.git
cd ~/tb4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_lab04 --symlink-install
source install/setup.bash
```

Mỗi terminal mới đều cần: `source /opt/ros/humble/setup.bash && source ~/tb4_ws/install/setup.bash`.

> Tài liệu tham khảo: [turtlebot4_simulator](https://github.com/turtlebot/turtlebot4_simulator), [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/). Nếu tên package/launch file trên máy khác với README (khác phiên bản), dùng `ros2 pkg list | grep turtlebot4` và `ros2 launch <package> <file> --show-args` để kiểm tra.

## 3. Phần 1 – Chạy TurtleBot 4 trong Gazebo

**Terminal 1:**
```bash
ros2 launch tb4_lab04 sim.launch.py world:=maze rviz:=true
# world: maze | warehouse | depot,  model: standard | lite
```
Bấm nút ▶ (Play) ở góc dưới trái cửa sổ Gazebo nếu mô phỏng đang tạm dừng.

**Terminal 2 – kiểm tra topic:**
```bash
./scripts/check_topics.sh       # hoặc làm tay bên dưới
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 topic info /cmd_vel
```
Gửi lệnh vận tốc bằng teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Kiểm tra: robot xuất hiện trong Gazebo, `/scan` và `/odom` có dữ liệu, robot chạy theo lệnh `/cmd_vel`, RViz2 hiển thị robot và tia LiDAR.

> Nếu topic nằm trong namespace (ví dụ `/tb4_1/scan`), dùng namespace đó cho mọi lệnh bên dưới (xem `ros2 topic list`).

## 4. Phần 2 – Xây dựng bản đồ bằng SLAM Toolbox

Giữ Terminal 1 (mô phỏng). Nếu Phần 1 đã mở RViz2 thì không cần mở thêm; `mapping.launch.py` mở RViz2 với cấu hình quan sát map.

**Terminal 2:**
```bash
ros2 launch tb4_lab04 mapping.launch.py
```
**Terminal 3 – lái robot khảo sát toàn bộ khu vực:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Lái chậm, đi dọc các bức tường, quay vòng ở ngã rẽ, đi qua mọi hành lang. Theo dõi bản đồ trên RViz2; nếu thấy tường bị nhân đôi/lệch thì dừng, lái lại qua vùng đó (loop closure) hoặc làm lại từ đầu.

**Lưu bản đồ khi đã đủ (Terminal 4):**
```bash
./scripts/save_map.sh            # tạo maps/map.yaml và maps/map.pgm
# hoặc thủ công:
ros2 run nav2_map_server map_saver_cli -f ~/tb4_ws/src/tb4_lab04/maps/map
```
Sau đó commit `maps/map.yaml` và `maps/map.pgm` lên repo.

## 5. Phần 3 – Localization

1. Dừng SLAM (Ctrl+C ở Terminal 2) và dừng mô phỏng (Ctrl+C ở Terminal 1).
2. Khởi động lại mô phỏng **cùng môi trường (cùng `world`)**:
   ```bash
   ros2 launch tb4_lab04 sim.launch.py world:=maze
   ```
3. Chạy localization với bản đồ vừa lưu:
   ```bash
   ros2 launch tb4_lab04 localization.launch.py map:=$HOME/tb4_ws/src/tb4_lab04/maps/map.yaml
   ```
4. Trên RViz2 dùng **2D Pose Estimate** đặt vị trí ban đầu của robot cho khớp với Gazebo, lái nhẹ robot vài vòng để particle cloud hội tụ.
5. Kiểm tra: robot trên bản đồ trùng vị trí thực tế trong Gazebo, tia LiDAR trùng tường trên bản đồ.

## 6. Phần 4 – Navigation với Nav2

Giữ mô phỏng + localization, mở Terminal mới:
```bash
ros2 launch tb4_lab04 navigation.launch.py
```
Trên RViz2 chọn **Nav2 Goal**, bấm và kéo tại một vị trí trống trên bản đồ. Robot tự lập kế hoạch và di chuyển tới goal; **không** điều khiển thủ công sau khi đặt goal. Đặt ít nhất 3 goal ở các khu vực khác nhau.

## 7. Xử lý sự cố thường gặp

| Hiện tượng | Cách xử lý |
|---|---|
| Không thấy `/scan` hoặc `/odom` | Bấm Play trong Gazebo; kiểm tra `ros2 topic list`; chờ ~30 s sau khi mở |
| Map không cập nhật / TF lỗi | Chạy đúng `use_sim_time:=true` (các launch của repo đã truyền sẵn); mỗi lần chỉ chạy một bản mô phỏng |
| Chạy lại bị trùng tiến trình | `pkill -9 -f "ign|gz|ruby|rviz2"` rồi chạy lại |
| Robot không bám map khi localization | Đặt lại **2D Pose Estimate** trên RViz2 |
| Nav2 báo goal thất bại | Đặt goal vào vùng trống, đã nằm trong bản đồ, cách tường đủ xa |

## 8. Nội dung nộp

- Video demo (Google Drive, công khai): _(dán link vào báo cáo)_
- Repo GitHub công khai: https://github.com/Hiep-h/tb4_lab04 (source, launch, `maps/map.yaml`, `maps/map.pgm`, README)
- Báo cáo PDF ngắn.
