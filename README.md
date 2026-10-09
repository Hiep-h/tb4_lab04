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
│   ├── navigation.launch.py     # Phần 4: Nav2
│   ├── survey.launch.py         # (tuỳ chọn) robot tự lái khảo sát để vẽ map
│   ├── drive_mapping.launch.py  # MỘT LỆNH: Gazebo + SLAM + RViz2 + bảng điều khiển (lái tay, tự lưu map)
│   └── auto_mapping.launch.py   # MỘT LỆNH: robot TỰ KHÁM PHÁ vẽ map nhanh, tự lưu khi xong
├── maps/                        # map.yaml + map.pgm (sinh ở Phần 2)
├── config/                      # cấu hình tuỳ chỉnh (nếu có)
├── scripts/
│   ├── check_topics.sh          # kiểm tra /scan, /odom, /cmd_vel
│   └── save_map.sh              # lưu bản đồ vào maps/
├── tb4_lab04/
│   ├── auto_explore.py          # node tự khám phá theo biên (frontier), tự lưu map khi xong
│   ├── explorer.py, frontier.py # thuật toán khám phá (thuần Python, kiểm thử được không cần ROS)
│   ├── control_panel.py         # bảng điều khiển thanh trượt (Tkinter) thay teleop bàn phím
│   ├── auto_survey.py           # node tự lái khảo sát (thay teleop) bằng LiDAR
│   └── wall_follow.py           # thuật toán bám tường bên phải (không phụ thuộc ROS)
├── package.xml, setup.py, setup.cfg
└── README.md
```

## 2. Cài đặt

Yêu cầu: Ubuntu 22.04 (hoặc WSL2 + Ubuntu 22.04), ROS 2 Humble Desktop.

```bash
sudo apt update
sudo apt install -y python3-tk ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
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

**World `small_house` (nhà ở AWS RoboMaker, bản nhẹ):** giữ tường + sàn và các đồ nội thất chính bằng mesh gốc (giường, sofa, tủ bếp,
tủ lạnh, bàn ăn + ghế, kệ TV...), bỏ đồ trang trí/cửa/đèn/rèm/thảm/trần/bóng cho nhẹ. Nếu máy vẫn lag, dùng `world:=small_house_lite`
(đồ nội thất thay bằng hộp, nhẹ nhất). Cài một lần (cần `sudo`), rồi chạy:
```bash
~/tb4_ws/src/tb4_lab04/scripts/install_world.sh
ros2 launch tb4_lab04 auto_mapping.launch.py world:=small_house x:=1.0 y:=1.0
```
Nguồn: [aws-robotics/aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world) (MIT-0, xem `models/LICENSE-AWS-small-house`).

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

**Robot tự khám phá và vẽ map (nhanh nhất, không cần lái):**
```bash
ros2 launch tb4_lab04 auto_mapping.launch.py world:=maze
```
Mở Gazebo, SLAM Toolbox + RViz2, rồi node `auto_explore`: robot xoay một vòng lấy bản đồ ban đầu, sau đó liên tục chọn **vùng chưa biết gần nhất** (biên: ô trống kề ô chưa biết), lập đường đi ngắn nhất tránh vật cản (Dijkstra trên bản đồ đã phồng quanh tường) và bám đường đi, lặp lại cho tới khi hết vùng chưa biết rồi dừng và **tự lưu** `maps/map.yaml` + `maps/map.pgm`. Log mỗi 5 giây in trạng thái, vị trí, % ô đã biết và số tia LiDAR hợp lệ; nếu LiDAR không thấy gì sẽ có cảnh báo. Tham số: `vmax:=0.3` (m/s), `map_dir:=<thư mục>`, `max_duration:=<giây>`. Đề bài ghi khảo sát bằng teleoperation; nếu nộp bằng chế độ tự khám phá, hãy nêu rõ trong báo cáo/video (hoặc dùng `drive_mapping.launch.py` để lái tay).

**Cách một lệnh mở tất cả, bạn tự lái bằng bảng thanh trượt:**
```bash
ros2 launch tb4_lab04 drive_mapping.launch.py world:=maze
```
Lệnh này mở Gazebo, sau ~25 giây mở SLAM Toolbox + RViz2 (bản đồ hiện ra và lớn dần khi robot đi), sau ~32 giây mở bảng điều khiển. Bạn kéo thanh **Tiến/Lùi** và **Xoay tròn** để lái. Phần **[ BẢN ĐỒ ]** trong bảng: nút *LƯU BẢN ĐỒ NGAY*, tự lưu mỗi 120 giây, và tự lưu khi đóng bảng (cả ba ghi vào `maps/map.yaml` + `maps/map.pgm`). Kéo thanh Tiến/Lùi sang **phải** để tiến, sang trái để lùi. Create 3 giới hạn việc lùi (log báo "Reached backup limit"), nên launch tự đặt `safety_override:=full` cho `/motion_control` (chỉ dùng trong mô phỏng). Khi lái xong chỉ cần **đóng cửa sổ bảng điều khiển** để bản đồ được lưu lần cuối, rồi Ctrl+C terminal. Tham số: `map_dir:=<thư mục>`, `autosave_period:=<giây>`, `max_linear`, `max_angular`.

**Cách điều khiển bằng bảng thanh trượt (không cần gõ phím trên terminal):**
```bash
ros2 run tb4_lab04 control_panel
ros2 run tb4_lab04 control_panel --ros-args -p max_linear:=0.3 -p max_angular:=1.0   # chỉnh tốc độ tối đa
```
Bảng có thanh **Tiến / Lùi** (m/s) và **Xoay tròn** (rad/s), nút đỏ **PHANH KHẨN CẤP** (phím SPACE), phím ↑↓←→ chỉnh nhanh, tuỳ chọn *Chống va chạm* (chặn tiến khi vật cản phía trước gần hơn 0,30 m, đọc `/scan`) và *Tự về 0 khi thả thanh trượt*. Tick *Tự lái (bám tường)* để robot tự đi khảo sát. Đây vẫn là điều khiển bằng tay (teleoperation) nên phù hợp yêu cầu của đề. TurtleBot 4 là robot hai bánh vi sai nên không có thanh trượt ngang như xe Mecanum.

**Cách khác – robot tự lái (không cần gõ phím):** thay Terminal 3 bằng node `auto_survey`, robot tự bám tường bên phải bằng LiDAR, đi hết một vòng rồi tự dừng khi quay lại gần điểm xuất phát (hoặc hết thời gian):
```bash
colcon build --packages-select tb4_lab04 --symlink-install && source install/setup.bash   # build lại 1 lần để có node mới
ros2 launch tb4_lab04 survey.launch.py                    # mặc định speed:=0.18 target_dist:=0.55 min_path:=15.0
ros2 launch tb4_lab04 survey.launch.py speed:=0.15 min_path:=25.0   # ví dụ chỉnh tham số
```
Robot xuất phát ở vùng trống thì đi thẳng để tìm tường, khi chạm tường thì quay trái cho tường nằm bên phải rồi bám theo. Node đặt mốc tại nơi bắt đầu bám tường ổn định (>5 s) và tự dừng khi đã đi đủ `min_path` mét, từng đi xa mốc ít nhất `min_excursion` (mặc định 2,5 m) và quay lại gần mốc (vòng bám tường khép kín), hoặc hết `max_duration`. Có thể chỉnh các tham số này bằng `--ros-args -p min_excursion:=3.0`. Log mỗi 5 giây cho biết trạng thái, quãng đường đã đi và khoảng cách xa xuất phát tối đa. Ctrl+C dừng robot ngay. Node chỉ đọc `/scan`, `/odom` và publish `/cmd_vel`. Lưu ý: đề bài yêu cầu khảo sát bằng teleoperation, nên nếu nộp bằng chế độ tự lái thì hãy nêu rõ trong báo cáo/video; khi map lệch có thể quay lại dùng teleop.

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
| `/scan` toàn 0.0, bản đồ SLAM rỗng, "Vật cản trước" luôn 12,00 m | LiDAR của Gazebo (GPU) dựng sai trên một số card Intel. Các launch của repo đã bật `LIBGL_ALWAYS_SOFTWARE=1` mặc định (`software_render:=false` để tắt). Chạy tay thì `export LIBGL_ALWAYS_SOFTWARE=1` trước `ros2 launch`; mô phỏng sẽ chậm hơn |
| Không thấy `/scan` hoặc `/odom` | Bấm Play trong Gazebo; kiểm tra `ros2 topic list`; chờ ~30 s sau khi mở |
| Map không cập nhật / TF lỗi | Chạy đúng `use_sim_time:=true` (các launch của repo đã truyền sẵn); mỗi lần chỉ chạy một bản mô phỏng |
| Chạy lại bị trùng tiến trình | `pkill -9 -f "ign|gz|ruby|rviz2"` rồi chạy lại |
| Robot không bám map khi localization | Đặt lại **2D Pose Estimate** trên RViz2 |
| Nav2 báo goal thất bại | Đặt goal vào vùng trống, đã nằm trong bản đồ, cách tường đủ xa |

## 8. Nội dung nộp

- Video demo (Google Drive, công khai): _(dán link vào báo cáo)_
- Repo GitHub công khai: https://github.com/Hiep-h/tb4_lab04 (source, launch, `maps/map.yaml`, `maps/map.pgm`, README)
- Báo cáo PDF ngắn.
