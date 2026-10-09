# maps/

`map.yaml` + `map.pgm`: bản đồ căn nhà `small_house` (độ phân giải 0,05 m, gốc khung `map` = chỗ robot xuất phát lúc vẽ map).

- `map.pgm` sinh ra ở Phần 2 (SLAM Toolbox, robot tự dò bằng `auto_mapping.launch.py`), sau đó làm sạch bằng
  `python3 tools/clean_map.py maps/map.pgm`: thân đồ nội thất (giường, sofa, tủ bếp…) là khối đặc mà LiDAR không nhìn xuyên qua
  nên bên trong còn "chưa biết" (xám); công cụ điền chúng thành vật cản. Vùng chưa biết bên ngoài tường giữ nguyên.
- `free_thresh: 0.196` để Nav2 hiểu pixel 205 là "chưa biết" (mặc định 0.25 sẽ coi là trống).
