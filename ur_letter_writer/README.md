# ur_letter_writer

Package ROS 2 (Humble) điều khiển UR3/UR3e vẽ **chữ cái đầu tiên trong tên** bằng
chuyển động của đầu công tác, dùng **MoveIt 2** để lập kế hoạch và thực thi
Cartesian trajectory trong Gazebo (`ur_simulation_gz`).

## 1. Ý tưởng thiết kế

- Mỗi chữ cái (A-Z) được định nghĩa bằng 1 hoặc nhiều **nét** (stroke).
- Mỗi nét là một danh sách điểm `(u, v)` trong hệ toạ độ chuẩn hoá `[0,1] x [0,1]`
  (dựa trên lưới 3x3 điểm: góc, cạnh giữa, tâm — giống font "vector đơn giản").
- `(u, v)` được ánh xạ sang toạ độ Cartesian thật thông qua **một mặt phẳng tuỳ
  chọn**, định nghĩa bởi:
  - `plane_center` (x, y, z): tâm mặt phẳng viết chữ
  - `u_axis`, `v_axis`: 2 trục ngang/dọc của chữ (trực giao hoá tự động)
  - `normal = u_axis × v_axis`: hướng pháp tuyến, dùng để "nhấc bút"
- Giữa 2 nét, đầu công tác được **lùi ra theo hướng normal** một khoảng
  `retract_distance` rồi mới di chuyển sang nét kế tiếp (tránh vẽ đè khi di
  chuyển giữa các nét).
- Toàn bộ đường đi (trừ bước tiếp cận đầu tiên, làm bằng joint-space plan) được
  lập kế hoạch bằng `move_group.computeCartesianPath()` — có kiểm tra
  self-collision và tự động từ chối nếu vượt giới hạn khớp (giảm `fraction`).
- Đường đi dự kiến của đầu công tác được publish dưới dạng `visualization_msgs/Marker`
  (topic `letter_writer/end_effector_path`) để quan sát hình dạng chữ trên RViz.

## 2. Cấu trúc package

```
ur_letter_writer/
├── package.xml
├── CMakeLists.txt
├── src/
│   └── letter_writer_node.cpp   # control node (font A-Z + MoveIt Cartesian path)
├── launch/
│   └── ur3_letter_writer.launch.py   # UR3/UR3e sim + MoveIt + control node
└── README.md
```

## 3. Cài đặt

Copy thư mục `ur_letter_writer` vào **cùng workspace** đã cài
`Universal_Robots_ROS2_Driver` và `Universal_Robots_ROS2_GZ_Simulation`
(ví dụ `~/ur_ws/src/`):

```bash
cp -r ur_letter_writer ~/ur_ws/src/
cd ~/ur_ws
rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install --packages-select ur_letter_writer
source install/setup.bash
```

> Nếu bạn cài UR packages qua apt (`ros-humble-ur`, `ros-humble-ur-simulation-gz`)
> thay vì build từ source, chỉ cần tạo workspace riêng chứa mỗi
> `ur_letter_writer` rồi build là đủ, vì package chỉ *phụ thuộc* vào
> `ur_simulation_gz` / `ur_moveit_config` đã có sẵn trong hệ thống.

## 4. Chạy mô phỏng

```bash
ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3e name:=Son
```

Launch file sẽ:
1. Khởi động UR3e trong Gazebo + MoveIt2 + RViz (dùng lại
   `ur_simulation_gz/launch/ur_sim_moveit.launch.py` có sẵn — **không** viết
   lại simulator/controller/MoveIt config).
2. Đợi `startup_delay` giây (mặc định 12s) để Gazebo + controller_manager +
   MoveIt sẵn sàng.
3. Chạy `letter_writer_node`: trích chữ cái đầu từ `name` (ở đây là `S`), lập
   kế hoạch và thực thi trajectory vẽ chữ.

Vài ví dụ khác:

```bash
# Dùng UR3 (CB-series) thay vì UR3e
ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3 name:=Minh

# Ghi đè trực tiếp chữ cần vẽ (bỏ qua "name")
ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3e letter:=A

# Tuỳ chỉnh kích thước và vị trí mặt phẳng vẽ
ros2 launch ur_letter_writer ur3_letter_writer.launch.py \
  name:=Bao letter_width:=0.25 letter_height:=0.3 \
  plane_center_x:=0.4 plane_center_z:=0.35
```

## 5. Danh sách tham số của `letter_writer_node`

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `student_name` | `"Son"` | Tên sinh viên, lấy ký tự chữ cái đầu tiên |
| `letter` | `""` | Ghi đè trực tiếp chữ cần vẽ (A-Z), ưu tiên hơn `student_name` |
| `planning_group` | `"ur_manipulator"` | Tên planning group trong MoveIt (theo `ur_moveit_config`) |
| `plane_center_x/y/z` | `0.35, 0.0, 0.30` | Tâm mặt phẳng vẽ chữ (m), trong planning frame |
| `plane_u_axis_x/y/z` | `0, 1, 0` | Trục "ngang" của chữ |
| `plane_v_axis_x/y/z` | `0, 0, 1` | Trục "dọc" của chữ (không cần vuông góc tuyệt đối với u — tự động trực giao hoá) |
| `letter_width` | `0.18` (m) | Bề rộng chữ |
| `letter_height` | `0.22` (m) | Bề cao chữ |
| `retract_distance` | `0.05` (m) | Khoảng cách "nhấc bút" giữa các nét |
| `eef_step` | `0.005` (m) | Bước nội suy Cartesian (càng nhỏ càng mượt, càng chậm lập kế hoạch) |
| `min_fraction` | `0.95` | Ngưỡng tối thiểu của `fraction` trả về từ `computeCartesianPath`, dưới ngưỡng này sẽ báo lỗi và **không** thực thi |
| `velocity_scaling` / `acceleration_scaling` | `0.2` | Giảm tốc độ để chuyển động rõ ràng, an toàn khi quan sát |

Mặc định `u_axis = Y`, `v_axis = Z` ⇒ mặt phẳng vẽ **thẳng đứng**, đối diện
robot theo hướng `+X` (giống viết lên một tấm bảng ảo đặt trước robot). Bạn có
thể đổi sang mặt phẳng nằm ngang (ví dụ "viết trên bàn") bằng cách đặt
`u_axis = (1,0,0)`, `v_axis = (0,1,0)` và giảm `plane_center_z` xuống gần độ
cao "mặt bàn" mong muốn.

## 6. Font chữ (A-Z)

Font dùng lưới 3 x 3 điểm chuẩn (góc trên/dưới trái/phải, giữa cạnh, tâm) và
chỉ vẽ bằng đoạn thẳng (không cong) để đơn giản, dễ debug. Có thể mở rộng/sửa
hình dạng từng chữ trong hàm `buildLetterLibrary()` của
`src/letter_writer_node.cpp`.

## 7. Phân tích chi tiết: chữ S được vẽ như thế nào (ví dụ tên "Son")

Với `name:=Son`, hàm `resolveLetter()` duyệt từng ký tự của chuỗi, gặp ký tự
chữ cái đầu tiên là `S` → in hoa → `letter = 'S'`. Chạy:

```bash
ros2 launch ur_letter_writer ur3_letter_writer.launch.py ur_type:=ur3e name:=Son
```

### 7.1. Định nghĩa nét (stroke) trong code

```cpp
lib['S'] = {{
  {0.85, 0.90}, {0.50, 1.00}, {0.15, 0.90},
  {0.10, 0.72}, {0.25, 0.58}, {0.50, 0.50},
  {0.75, 0.42}, {0.90, 0.28}, {0.85, 0.10},
  {0.50, 0.00}, {0.15, 0.10}
}};
```

Đây là **một mảng chứa đúng 1 stroke duy nhất** (`shape.size() == 1`), gồm
**11 điểm** `(u, v)` chuẩn hoá trong `[0,1] x [0,1]`. Vì chỉ có 1 stroke nên
**không có lần "nhấc bút" nào ở giữa** — cả chữ S được vẽ bằng **một nét liền
mạch**, giống khi viết tay không nhấc bút.

### 7.2. Vì sao 11 điểm này tạo thành hình chữ S

Nếu lấy tâm hình là `(0.5, 0.5)`, 11 điểm này **đối xứng qua tâm 180°** từng
cặp — đây chính là đặc điểm hình học chuẩn của chữ S trong hầu hết các font
(nửa trên và nửa dưới là ảnh xoay 180° của nhau):

| STT | (u, v) | Vai trò trong hình | STT đối xứng |
|---|---|---|---|
| 1 | (0.85, 0.90) | Điểm bắt đầu — góc trên-phải, đỉnh vòng cung trên | ↔ 11 |
| 2 | (0.50, 1.00) | Đỉnh trên cùng, chính giữa | ↔ 10 |
| 3 | (0.15, 0.90) | Góc trên-trái — vòng cung trên bắt đầu đổ xuống | ↔ 9 |
| 4 | (0.10, 0.72) | Mép trái, giữa vòng cung trên | ↔ 8 |
| 5 | (0.25, 0.58) | Điểm thắt vào gần tâm, chuẩn bị đổi hướng cong | ↔ 7 |
| 6 | (0.50, 0.50) | **Tâm hình** — điểm uốn (inflection point) nối 2 vòng cung | ↔ 6 (chính nó) |
| 7 | (0.75, 0.42) | Điểm thắt đối xứng, bắt đầu vòng cung dưới | ↔ 5 |
| 8 | (0.90, 0.28) | Mép phải, giữa vòng cung dưới | ↔ 4 |
| 9 | (0.85, 0.10) | Góc dưới-phải — vòng cung dưới | ↔ 3 |
| 10 | (0.50, 0.00) | Đáy, chính giữa | ↔ 2 |
| 11 | (0.15, 0.10) | Điểm kết thúc — góc dưới-trái | ↔ 1 |

Vì chỉ nối bằng **đoạn thẳng** (không có cung tròn thật), 11 điểm (thay vì 6
điểm ở font block gốc) giúp xấp xỉ 2 vòng cong mượt hơn — mỗi điểm thêm là
một "khúc gãy" nhỏ, càng nhiều điểm thì càng gần với đường cong chữ S thật.

### 7.3. Chuyển từ `(u,v)` sang toạ độ thật ngoài không gian

Với tham số mặc định (`plane_center=(0.35, 0, 0.30)`, `u_axis=(0,1,0)`,
`v_axis=(0,0,1)`, `letter_width=0.18`, `letter_height=0.22`), hàm `poseAt()`
tính:

```
offset_u = (u - 0.5) * letter_width
offset_v = (v - 0.5) * letter_height
pos = plane_center + offset_u * u_axis + offset_v * v_axis
```

Toạ độ Cartesian (mét) khi đầu công tác **thực sự chạm** vào từng điểm:

| STT | (u, v) | x | y | z |
|---|---|---|---|---|
| 1 | (0.85, 0.90) | 0.35 | 0.063 | 0.388 |
| 2 | (0.50, 1.00) | 0.35 | 0.000 | 0.410 |
| 3 | (0.15, 0.90) | 0.35 | -0.063 | 0.388 |
| 4 | (0.10, 0.72) | 0.35 | -0.072 | 0.348 |
| 5 | (0.25, 0.58) | 0.35 | -0.045 | 0.318 |
| 6 | (0.50, 0.50) | 0.35 | 0.000 | 0.300 |
| 7 | (0.75, 0.42) | 0.35 | 0.045 | 0.282 |
| 8 | (0.90, 0.28) | 0.35 | 0.072 | 0.252 |
| 9 | (0.85, 0.10) | 0.35 | 0.063 | 0.212 |
| 10 | (0.50, 0.00) | 0.35 | 0.000 | 0.190 |
| 11 | (0.15, 0.10) | 0.35 | -0.063 | 0.212 |

Cột `x` luôn cố định `0.35` vì mặt phẳng vẽ là mặt phẳng đứng vuông góc trục
X (`normal_axis = u_axis × v_axis = (1,0,0)`); chữ "vẽ" hoàn toàn trên mặt
phẳng Y-Z này. `y` đóng vai trò bề ngang chữ, `z` đóng vai trò chiều cao.

### 7.4. Trình tự chuyển động thực tế (2 giai đoạn)

Trong `run()`, vì `S` chỉ có 1 stroke, đoạn dựng `sequence` rút gọn thành:

1. **Tiếp cận** (retracted=true): lùi `0.05 m` theo `normal_axis=(1,0,0)` từ
   điểm 1 → vị trí `(0.30, 0.063, 0.388)`.
2. **Hạ bút** xuống đúng điểm 1: `(0.35, 0.063, 0.388)`.
3. **Vẽ liên tục** qua điểm 2 → 11 (đều `retracted=false`, không nhấc bút).
4. **Nhấc bút** ở cuối: lùi `0.05 m` từ điểm 11 → `(0.30, -0.063, 0.212)`.

→ **Giai đoạn 1** (`move_group.setPoseTarget` + `plan()` + `execute()`):
robot di chuyển (joint-space, không ràng buộc đường thẳng) từ vị trí hiện tại
đến điểm tiếp cận ở bước 1.

→ **Giai đoạn 2** (`move_group.computeCartesianPath()` + `execute()`): 12
pose còn lại (bước 2 → 4) được nối thành **một trajectory Cartesian duy
nhất**, robot di chuyển thẳng qua từng điểm theo đúng thứ tự — đây chính là
đoạn vẽ nên hình chữ S mà bạn thấy trên marker màu xanh dương trong RViz.

## 8. Xem đường đã vẽ trên RViz

Node publish liên tục (mỗi giây, cho tới khi bạn nhấn **Ctrl+C** ở terminal
chạy launch file) một `visualization_msgs/MarkerArray` trên topic
**`letter_writer/markers`**, gồm 2 marker:

- `letter_writer_draw` (màu **xanh dương**, nét dày): các đoạn đầu công tác
  **thực sự chạm mặt phẳng** — chính là hình chữ đã vẽ.
- `letter_writer_lift` (màu **xám nhạt, mờ**): các đoạn di chuyển lúc **nhấc
  bút** giữa các nét — không phải một phần của chữ.

Vì RViz mặc định của `ur_sim_moveit.launch.py` **không có sẵn** Display cho
topic này, bạn cần tự thêm:

1. Trong cửa sổ RViz, ở panel **Displays** bên trái, bấm nút **Add**.
2. Chọn tab **By topic**, tìm `letter_writer/markers` → chọn **MarkerArray** → **OK**.
3. Kiểm tra **Fixed Frame** (Global Options ở đầu panel Displays) đang đặt
   đúng planning frame của robot (thường là `base_link`) — nếu để sai frame,
   marker sẽ không hiển thị dù đã Add đúng topic.

Vì node vẫn publish lại mỗi giây, bạn có thể Add Display **bất cứ lúc nào**
(trước, trong, hoặc sau khi robot vẽ xong) và vẫn thấy được đường đi.

## 9. Xử lý lỗi thường gặp

- Nếu terminal báo **"Lập kế hoạch đến vị trí tiếp cận thất bại"**: chỉnh
  `plane_center_x/y/z` gần robot hơn hoặc đổi hướng mặt phẳng — vị trí hiện
  tại nằm ngoài vùng làm việc hoặc gây self-collision.
- Nếu terminal báo **"Fraction thấp hơn ngưỡng yêu cầu"**: giảm
  `letter_width`/`letter_height`, hoặc đưa `plane_center` gần robot hơn — chữ
  đang quá lớn/quá xa khiến một số điểm vượt giới hạn khớp hoặc IK thất bại.
- Nếu robot vẽ xong nhưng **RViz không hiện gì**: kiểm tra lại mục 8 ở trên
  (đã Add MarkerArray đúng topic chưa, Fixed Frame có đúng không).

## 10. Hướng mở rộng (không bắt buộc)

Có thể thay `buildLetterLibrary()` bằng pipeline trích xuất waypoint từ ảnh:

```
Ảnh chữ → Threshold/Binarization → Contour Extraction → Raw 2D Path →
Simplify/Resample → Scale + Translate → Pixel (u,v) → Cartesian (x,y,z) →
Cartesian Waypoints → MoveIt → UR3
```

Có thể viết một node Python riêng (OpenCV) xuất ra file YAML/JSON danh sách
stroke `(u,v)`, rồi sửa `letter_writer_node` đọc file đó thay vì bảng tra cứu
cứng trong code.
