# maps/

- `map.yaml` + `map.pgm`: **bản đồ gốc do SLAM Toolbox sinh ra** (robot tự dò bằng `auto_mapping.launch.py`, lưu bằng `map_saver_cli`),
  căn nhà `small_house`, độ phân giải 0,05 m, gốc khung `map` = chỗ robot xuất phát lúc vẽ. `free_thresh: 0.196` để Nav2 hiểu
  pixel 205 là "chưa biết" (mặc định 0.25 sẽ coi là trống).
- `map_clean.yaml` + `map_clean.pgm` (tùy chọn): cùng bản đồ nhưng đã xử lý hậu kỳ bằng `python3 tools/clean_map.py maps/map.pgm maps/map_clean.pgm`.
  Thân đồ nội thất (giường, sofa, tủ bếp…) là khối đặc mà LiDAR không nhìn xuyên qua nên bên trong còn "chưa biết" (xám); công cụ điền
  chúng thành vật cản. Vùng chưa biết ngoài tường giữ nguyên. Dùng bản này bằng `map:=<đường dẫn>/map_clean.yaml`.
