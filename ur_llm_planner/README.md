# ur_llm_planner — Điều khiển UR3/UR3e bằng LLM, Camera và Skill-based Planning

Package ROS 2 Humble cho phép điều khiển UR3/UR3e (mô phỏng Gazebo + MoveIt 2) bằng câu lệnh
ngôn ngữ tự nhiên. LLM (kết nối qua **9Router**) **chỉ** hiểu yêu cầu và chọn/sắp xếp các robot
skill. LLM **không** sinh joint trajectory hay giá trị khớp. Vị trí vật và trạng thái các vùng
được xác định bằng **camera**, không khai báo sẵn.

```
                    Camera (Gazebo) ──► perception_node ──► /perception/scene ──────────────┐
                                                                                           ▼
Natural Language ──► LLM ──► JSON Plan ──► Plan Resolver ──► Plan Validator ──► Skill Server ──► MoveIt 2 ──► UR3/UR3e
   Command        (9Router)                (hệ thống chèn     (whitelist +        (/execute_skill)
                     ▲                      check_zone +       trạng thái vùng)        │
                     │                      bước dọn vùng)                             │
                     └───────────────────────────── /scene_state ◄────────────────────┘
                              (check_zone khác dự kiến → lập kế hoạch lại)
```

## 1. Kiến trúc

| Thành phần | File | Vai trò |
|---|---|---|
| Môi trường Gazebo | `worlds/pick_place.sdf` | Bàn, camera trên cao, 5 khối (vật thể động), 3 vùng `zone_a/b/c` |
| Robot + gripper | `urdf/ur_robotiq.urdf.xacro` | UR3/UR3e + Robotiq 2F-85, TCP `grasp_tcp`, DetachableJoint cho từng khối |
| MoveIt SRDF | `srdf/ur_robotiq.srdf.xacro` | SRDF của UR + tắt va chạm giữa các khâu gripper |
| Controller gripper | `config/gripper_controller.yaml` | `gripper_controller` (JointTrajectoryController, 6 khớp ngón) |
| Khai báo scene | `config/scene.yaml` | Tên + màu của khối, vị trí 3 vùng, kích thước bàn/khối/vùng. **Không có vị trí khối** |
| Cấu hình camera | `config/perception.yaml` | Topic camera, dải màu HSV, ngưỡng lọc blob |
| Thuật toán nhận dạng | `llm_planner/perception.py` | Phân đoạn HSV, tâm mặt trên, chiếu tia → toạ độ world, gán vùng |
| Perception node | `scripts/perception_node.py` | Camera → `/perception/scene` (khối ở đâu, vùng nào trống / có vật) |
| Message | `msg/SceneObservation.msg`, `DetectedObject.msg`, `ZoneState.msg` | Kết quả quan sát của camera |
| Thông tin sinh viên | `config/student_config.yaml` | `student.name`, `student.id` → nhiệm vụ cá nhân |
| Nhiệm vụ cá nhân | `llm_planner/student.py` | `P = XX mod 6` xếp red/yellow/blue vào zone A/B/C |
| Skill Server (C++) | `src/skill_server.cpp` | Thực thi skill bằng MoveIt 2, vị trí vật lấy từ camera |
| Interface | `srv/ExecuteSkill.srv` | `skill, object, zone` → `status, message` |
| LLM client | `llm_planner/llm_client.py` | Gọi 9Router (API tương thích OpenAI) |
| Prompt | `llm_planner/prompt.py` | Mô tả skill/vật/vùng + trạng thái camera + quy tắc, yêu cầu trả về JSON |
| Plan Resolver | `llm_planner/plan_resolver.py` | Hệ thống tự chèn `check_zone` + bước dọn vùng bị chiếm nếu LLM bỏ sót |
| Plan Validator | `llm_planner/plan_validator.py` | Kiểm tra kế hoạch, từ chối skill/object/zone không hợp lệ, ghi trạng thái dự kiến cho `check_zone` |
| LLM Planner node | `scripts/llm_planner_node.py` | Câu lệnh → trạng thái camera → LLM → Resolver → Validator → gọi từng skill, đối chiếu `check_zone`, lập lại kế hoạch khi môi trường thay đổi |
| Launch | `launch/sim.launch.py` | Gazebo + ros2_control + MoveIt 2 + bridge + TF camera + perception + skill_server |

### 1.1. Robot Skills

Skill được chia hai nhóm. LLM chỉ chọn tên skill + tham số (`object`, `zone`); toạ độ do camera
và skill server xác định.

**Skill quan sát (camera)** — robot **đứng yên**, trả về kết quả dạng JSON (trường `data` của
`ExecuteSkill.srv`):

| Skill | Tham số | Mô tả |
|---|---|---|
| `detect_objects()` | – | Liệt kê mọi khối camera thấy, vị trí, vùng chứa khối |
| `check_zone(zone)` | zone | Vùng đang trống hay đang có khối nào (`{"zone", "occupied", "object"}`) |
| `find_object(object)` | object | Vị trí + vùng của một khối (chờ camera thấy lại khối) |
| `find_free_position()` | – | Tìm ô trống trên bàn: ngoài mọi vùng, cách các khối khác, robot với tới được (giải IK trước, không di chuyển). Lưu làm `temporary_position` |

**Skill chuyển động (MoveIt 2):**

| Skill | Tham số | Mô tả |
|---|---|---|
| `home()` | – | Về tư thế nghỉ phía sau robot (joint target, OMPL) — không che camera |
| `pick(object)` | object | `move_above(object)` + `close_gripper()` |
| `place(object, zone)` | object, zone | Kiểm tra đích còn trống (camera) → `move_to_zone(zone)` + `open_gripper()`. `zone` có thể là `temporary_position` |
| `move_above(object)` | object | Chờ camera thấy vật, đưa `grasp_tcp` (hướng xuống) tới phía trên tâm vật 12 cm |
| `move_to_zone(zone)` | zone | Đưa `grasp_tcp` tới phía trên vùng (hoặc `temporary_position`) |
| `close_gripper()` | – | Mở ngón, hạ thẳng xuống tâm vật, kẹp, **xác nhận ngón đã chạm vật**, khoá vật, nâng lên |
| `open_gripper()` | – | Hạ vật xuống mặt bên dưới, mở ngón, nhả vật, nâng lên |

`temporary_position` dùng được **một lần** cho mỗi lần `find_free_position()`: thả vật vào đó
xong thì vị trí đã bị chiếm, lần dời vật sau phải tìm vị trí mới. Vị trí tạm hiện trên RViz
(marker vàng, topic `/skill_server/markers`).

Mỗi skill trả về một trong các trạng thái:

`SUCCESS`, `FAILED`, `INVALID_SKILL`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED`,
`EXECUTION_FAILED`, `OBJECT_NOT_HELD`, `GRIPPER_BUSY`, `NO_OBJECT_TO_GRASP`,
`OBJECT_NOT_VISIBLE` (camera không thấy vật), `ZONE_OCCUPIED` (camera thấy đích đang có vật),
`NO_FREE_SPACE` (không còn ô trống nào robot với tới).

### 1.2. Camera và trạng thái môi trường

Camera RGB 640×480 cố định tại `(0.15, 0, 1.5)`, trục quang vuông góc mặt bàn (nhìn thẳng
xuống). `perception_node` xử lý mỗi khung hình (10 Hz):

1. **Phân đoạn màu HSV**: mỗi khối một dải màu (`config/perception.yaml`). Bàn (xám đậm), các
   vùng (xám sáng) và thân robot có độ bão hoà thấp nên không bị nhầm với khối.
2. **Lọc blob**: lấy thành phần liên thông lớn nhất, diện tích phải nằm trong
   `[0.6, 2.5] ×` diện tích kỳ vọng. Khối bị che một phần (blob nhỏ) không được dùng.
3. **Tâm mặt trên**: chỉ giữ các pixel sáng nhất của blob (mặt trên được chiếu sáng trực tiếp).
   Camera nhìn thấy thêm một mặt bên tối của khối — lấy cả blob thì tâm lệch vài mm.
4. **Chiếu tia**: tia từ tâm camera qua pixel `(u, v)` (dùng `camera_info.K` + TF
   `world → camera_optical_frame`) giao với mặt phẳng `z = cube_size` → `(x, y)` trong `world`.
5. **Gán vùng**: khối thuộc vùng nếu tâm khối cách tâm vùng ≤ `zone_size/2 + 0.01` m theo x và y.

Sai số đo được trong Gazebo: **≤ 1.5 mm** cho cả 5 khối.

- Khối bị che (ví dụ gripper đang ở ngay phía trên sau khi thả) được giữ vị trí quan sát gần
  nhất với `visible = false`. Khối vừa được gắp bị xoá vị trí cũ, sau khi thả sẽ báo "chưa
  thấy" cho tới khi camera nhìn lại được.
- Skill server dùng vị trí ước lượng lúc thả (`source: estimated`) cho tới khi camera xác nhận.
- Pose camera chỉ khai báo một nơi (`worlds/pick_place.sdf`); `sim.launch.py` đọc pose đó để
  publish TF tĩnh `world → camera_link → camera_optical_frame`.

**Kiểm tra môi trường trước mỗi hành động:**

- `llm_planner_node` in `SCENE (camera)` trước khi lập kế hoạch, và từ chối chạy robot nếu chưa
  có `/scene_state`.
- Hệ thống chèn `check_zone(Z)` trước mỗi lần đặt vật vào vùng `Z` (mục 1.7). Lúc thực thi,
  kết quả camera được so với trạng thái kế hoạch dự kiến; khác nhau → dừng, lập kế hoạch lại.
- `skill_server` chờ một khung hình camera mới trước **mỗi** skill và đồng bộ vị trí các khối
  vào planning scene của MoveIt. `move_above`/`pick` chờ camera thấy lại vật cần gắp.
  `place`/`open_gripper` kiểm tra đích còn trống ngay trước khi thả (`ZONE_OCCUPIED`).

### 1.3. An toàn chuyển động

- **Joint limit**: mọi quỹ đạo do MoveIt 2 lập kế hoạch theo `joint_limits.yaml` của
  `ur_moveit_config`. Khi giải IK, skill server còn giới hạn cấu hình (`|shoulder_pan| ≤ π`,
  elbow-up, cổ tay hướng xuống) để tránh các nghiệm "xoắn". Nghiệm IK còn phải không tự va chạm
  và không chạm bàn. Nếu OMPL vẫn không lập được kế hoạch tới nghiệm đó, skill server giải lại IK
  từ seed khác (tối đa 22 lần).
- **Self-collision**: kiểm tra bằng SRDF của `ur_moveit_config`.
- **Va chạm môi trường**: bàn và 5 khối (vị trí theo camera) là collision object trong planning
  scene. Khối đang cầm được *attach* vào `grasp_tcp`, nên MoveIt tính cả gripper và khối đó.
- Các đoạn hạ/nâng thẳng đứng dùng `computeCartesianPath(..., avoid_collisions = true)`. Nếu
  fraction < 98% thì trả về `PLANNING_FAILED` và **không** thực thi.
- **Khớp `wrist_3` (continuous)**: MoveIt chuẩn hoá góc về `[-π, π]` nhưng controller trong
  Gazebo dùng góc chưa chuẩn hoá (vd 4.78 rad sau khi quay qua mốc π) → quỹ đạo lệch đúng 2π,
  controller huỷ (`Position Error: -6.283185`). Skill server dịch quỹ đạo của khớp continuous
  đi `k·2π` cho điểm đầu trùng góc thực trong `/joint_states` trước khi gửi.

### 1.4. Gripper Robotiq 2F-85

Robotiq 2F-85 gắn tại `tool0`. Mô hình lấy từ gói `robotiq_description`.

- **TCP**: link `grasp_tcp` cách đế gripper 0.145 m, nằm ở tâm giữa hai ngón.
- **Ngón kẹp**: `ign_ros2_control` của Humble không hỗ trợ khớp mimic, nên `gripper_controller`
  điều khiển cả 6 khớp ngón. Skill server gửi góc `q` nhân với hệ số mimic của URDF
  (`gripper_mimic_multipliers`). Khe hở giữa hai má kẹp (đo bằng TF):

  | q (rad) | 0 | 0.43 | 0.45 | 0.50 |
  |---|---|---|---|---|
  | Khe hở | 85 mm | 42.0 mm | 39.8 mm | 34.3 mm |

- **Kẹp thật + xác nhận tiếp xúc**: lệnh kẹp `q = 0.50` (khe 34 mm < khối 40 mm), nên ngón ép vào
  khối và bị khối chặn lại ở ~0.45 rad. Skill server đọc `/joint_states`: sai lệch giữa góc lệnh
  và góc thực > `grasp_contact_tolerance` (0.02 rad) ⇒ đã chạm vật. Đo được 0.040–0.048 rad khi
  có khối, ~0.002 rad khi kẹp hụt. Kẹp hụt ⇒ `NO_OBJECT_TO_GRASP`, không khoá gì cả.
- **Giữ vật**: chỉ **sau khi** xác nhận tiếp xúc, skill server khoá khối vào `wrist_3_link` bằng
  plugin **DetachableJoint** của Gazebo (topic `/gripper/<vật>/attach`) để khối không trượt. Đây
  là ràng buộc vật lý, không dịch chuyển khối. Khi thả: mở ngón, nhả qua `/gripper/<vật>/detach`.
- **Không đặt pose vật trực tiếp**: không có lời gọi `set_pose` nào tới Gazebo (đã bỏ bridge
  `/world/.../set_pose`). Khối chỉ di chuyển do vật lý; vị trí mới do camera quan sát.
- Plugin DetachableJoint tự khoá mọi khối lúc khởi động, nên skill server nhả tất cả (các khối
  vẫn nằm nguyên chỗ).
- Trong MoveIt: `attachObject` / `detachObject` khối vào `grasp_tcp`, với `touch_links` là các khâu
  của gripper.

### 1.5. Plan Validator

Validator dùng cơ chế **whitelist**. Chỉ cần một bước sai là **toàn bộ** kế hoạch bị từ chối,
robot không chạy dù chỉ một phần:

- JSON phải có dạng `{"plan": [ ... ]}` và có tối đa **40** bước (5 khối × pick/place +
  `check_zone` + các bước dọn vùng + `home`).
- `skill` phải thuộc danh sách 11 skill ở trên.
- Mỗi bước chỉ được có đúng các tham số của skill đó. Mọi trường khác (`joints`, `trajectory`,
  `position`, `x`/`y`...) đều bị từ chối, nên LLM không thể điều khiển khớp trực tiếp.
- `object`/`zone` phải nằm trong `object_names`/`zone_names` của `scene.yaml`.
- `temporary_position` chỉ hợp lệ cho `place`/`move_to_zone`, và phải có `find_free_position()`
  chưa dùng đứng trước.
- Validator mô phỏng trạng thái gripper để bắt các thứ tự vô lý: `place` khi chưa `pick`, `pick`
  khi đang cầm vật khác, `close_gripper` mà không có `move_above` ngay trước đó (skill quan sát
  xen giữa không tính vì robot đứng yên)...
- Validator mô phỏng cả **trạng thái các vùng** (từ `/scene_state`, tức là từ camera): mỗi vùng
  chỉ chứa một vật. Kế hoạch đặt vật vào vùng đang có vật khác bị từ chối. Với mỗi
  `check_zone(Z)`, validator ghi lại khối **dự kiến** nằm ở `Z` tại thời điểm đó (để đối chiếu
  với camera lúc chạy).

Khi kế hoạch bị từ chối, planner gửi danh sách lỗi lại cho LLM để nó tự sửa (tối đa
`llm.max_repair_attempts` lần, mặc định 2). Kế hoạch sau khi sửa vẫn phải qua validator.
Skill server cũng kiểm tra lại skill/object/zone và vùng trống (phòng thủ hai lớp).

### 1.6. Cá nhân hoá theo MSSV

Khai báo trong `config/student_config.yaml` (nhớ thay bằng tên và MSSV **thật** của bạn):

```yaml
/**:
  ros__parameters:
    student:
      name: "Nguyen Van An"
      id: "23020123"
```

Hoặc ghi đè khi chạy: `--ros-args -p student.name:="Nguyen Van An" -p student.id:=23020123`.

Với `XX` là hai chữ số cuối MSSV, `P = XX mod 6` quyết định khối nào đặt vào zone A/B/C
(theo đề bài):

| P | Zone A | Zone B | Zone C |
|---|---|---|---|
| 0 | Red | Yellow | Blue |
| 1 | Red | Blue | Yellow |
| 2 | Yellow | Red | Blue |
| 3 | Yellow | Blue | Red |
| 4 | Blue | Red | Yellow |
| 5 | Blue | Yellow | Red |

Hai khối còn lại trong scene (green, purple) không thuộc nhiệm vụ nên phải nằm **ngoài** cả 3
vùng — nếu đang chiếm một vùng đích thì được dời ra ô trống trên bàn.

Ví dụ của đề: MSSV 23020123 → `P = 23 mod 6 = 5` → A = Blue, B = Yellow, C = Red.

MSSV 23020764 → `P = 64 mod 6 = 4` → A = Blue, B = Red, C = Yellow. Khi khởi động, planner in:

```
STUDENT: Vu Ngoc Son | ID: 23020764 | P = 64 mod 6 = 4
PERSONAL TASK: zone_a <- blue_cube | zone_b <- red_cube | zone_c <- yellow_cube
               ngoai cac vung (tren ban): green_cube, purple_cube
```

Bảng gán này được đưa vào system prompt, nên LLM tự lập kế hoạch cho các lệnh như
`Arrange all objects according to my student ID.`

### 1.7. Luồng xử lý: vùng đích bị chiếm

Trạng thái ban đầu của world đúng như ví dụ của đề: `blue_cube` nằm trong `zone_b`,
`green_cube` nằm trong `zone_c`, 3 khối còn lại (red, yellow, purple) nằm trên bàn.

Với lệnh `Put the red cube in Zone B.`, hệ thống thực hiện:

```
Camera → zone_b đang có blue_cube
  check_zone(zone_b)                        # camera xác nhận lại ngay trước khi thao tác
  pick(blue_cube)
  find_free_position()                      # ô trống trên bàn → temporary_position
  place(blue_cube, temporary_position)      # di chuyển vật chiếm chỗ ra ngoài
  pick(red_cube)
  place(red_cube, zone_b)
  home()
```

Có **ba lớp** bảo đảm kế hoạch xử lý vùng bị chiếm:

1. **LLM**: prompt có trạng thái từng vùng (từ camera) + quy tắc 7 + ví dụ few-shot giống hệt
   kịch bản trên.
2. **Plan Resolver** (`llm_planner/plan_resolver.py`, tham số `auto_resolve`, mặc định bật): với
   mỗi cặp `pick(X) … place(X, Z)`, hệ thống chèn `check_zone(Z)` nếu chưa có, và nếu `Z` đang có
   khối `Y` thì chèn `pick(Y) → find_free_position() → place(Y, temporary_position)`. Resolver chỉ
   **thêm** bước (đánh dấu `+` trên terminal), không xoá/đổi bước của LLM. Nếu LLM đã tự xử lý
   (kể cả chuyển `Y` sang vùng khác), resolver không chèn gì.
3. **Plan Validator**: kiểm tra lại toàn bộ kế hoạch sau khi bổ sung. Còn sai → gửi lỗi cho LLM
   sửa (`llm.max_repair_attempts`).

**Khi môi trường thay đổi trong lúc thực thi** (vd: có người đặt thêm khối vào vùng): kết quả
`check_zone` khác trạng thái dự kiến → dừng các bước còn lại (`SKIPPED`), đọc lại camera, gọi LLM
lập kế hoạch mới (tham số `max_replans`, mặc định 1). Lệnh gửi qua `/plan_json` (không có LLM)
thì dừng với `TASK FAILED`. Đã chạy thử: đặt `red_cube` vào `zone_c` giữa chừng → hệ thống phát
hiện, lập lại kế hoạch có thêm bước dọn `red_cube`.

Kế hoạch dự kiến cho MSSV 23020764 (A = blue, B = red, C = yellow) từ trạng thái ban đầu của
world — LLM chỉ cần sinh `pick → place` cho 3 khối, hệ thống tự thêm các bước `+`:

```
+ check_zone(zone_a)
  pick(blue_cube)
  place(blue_cube, zone_a)                  # blue rời zone_b -> zone_b trống
+ check_zone(zone_b)
  pick(red_cube)
  place(red_cube, zone_b)
+ check_zone(zone_c)
+ pick(green_cube)
+ find_free_position()
+ place(green_cube, temporary_position)     # green không thuộc nhiệm vụ, dọn zone_c
  pick(yellow_cube)
  place(yellow_cube, zone_c)
  home()
```

### 1.8. Đầu ra trên terminal

Kết quả chạy thật trong Gazebo cho lệnh demo (ở đây LLM chỉ sinh `pick → place → home`):

```
USER COMMAND:
Put the red cube in Zone B.

SCENE (camera):
- Gripper dang cam: khong co gi
- zone_a: trong
- zone_b: dang co blue_cube
- zone_c: dang co green_cube
- purple_cube: tren ban, ngoai moi vung
- red_cube: tren ban, ngoai moi vung
- yellow_cube: tren ban, ngoai moi vung

LLM PLAN:
    pick(red_cube)
    place(red_cube, zone_b)
    home()

HE THONG (kiem tra vung bang camera):
    - chen check_zone(zone_b) truoc khi dat 'red_cube' vao zone_b
    - zone_b dang co 'blue_cube' (camera) -> chen pick(blue_cube) -> find_free_position() -> place(blue_cube, temporary_position)

KE HOACH THUC THI (+ = buoc he thong them vao):
    + check_zone(zone_b)
    + pick(blue_cube)
    + find_free_position()
    + place(blue_cube, temporary_position)
      pick(red_cube)
      place(red_cube, zone_b)
      home()
VALIDATOR: plan hop le (7 buoc)

EXECUTION:
+ check_zone(zone_b) ..................... SUCCESS  zone_b: dang co blue_cube (camera)
+ pick(blue_cube) ........................ SUCCESS
+ find_free_position() ................... SUCCESS  temporary_position = (0.200, -0.000) (o trong theo camera, 11 o ung vien)
+ place(blue_cube, temporary_position) ... SUCCESS
pick(red_cube) ........................... SUCCESS
place(red_cube, zone_b) .................. SUCCESS
home() ................................... SUCCESS

TASK SUCCESS
```

Nếu một bước thất bại, các bước còn lại được đánh dấu `SKIPPED` và kết thúc bằng `TASK FAILED`.
Kế hoạch bị validator từ chối kết thúc bằng `TASK REJECTED`. Thêm `-p verbose:=true` để in câu
trả lời thô của LLM.

## 2. Cài đặt

Yêu cầu: Ubuntu 22.04 (hoặc WSL2 + Ubuntu 22.04), ROS 2 Humble, Gazebo Fortress, MoveIt 2,
`Universal_Robots_ROS2_Driver` và `ur_simulation_gz` (đã có trong workspace này). Camera cần
render OpenGL (ogre2).

```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-robotiq-description   # mô hình Robotiq 2F-85
rosdep install --ignore-src --from-paths src -y   # gồm python3-opencv, python3-numpy
colcon build --packages-select ur_llm_planner
source install/setup.bash
```

### 2.1. Cài và cấu hình 9Router

9Router là router LLM chạy cục bộ, cung cấp endpoint tương thích OpenAI.

1. Cài Node.js, rồi cài và chạy 9Router theo hướng dẫn của dự án 9Router (ví dụ
   `npm install -g 9router` rồi `9router`).
2. Mở dashboard 9Router: kết nối một provider/model, tạo API key, rồi ghi lại **base URL** (mặc
   định trong package là `http://localhost:20128/v1`) và **tên model/combo**.
3. Đặt API key bằng biến môi trường, không ghi key vào file:

   ```bash
   export NINEROUTER_API_KEY=<api_key_tu_dashboard>
   ```

4. Điền `model` (và `base_url` nếu khác) trong `config/llm.yaml`, hoặc truyền bằng
   `-p llm.model:=...` khi chạy.

> Planner gửi request chuẩn OpenAI `POST {base_url}/chat/completions`, nên có thể dùng bất kỳ
> endpoint tương thích OpenAI nào bằng cách đổi `llm.base_url`.

## 3. Chạy

**Terminal 1** — mô phỏng + camera + MoveIt 2 + skill server:

```bash
source ~/ur_ws/install/setup.bash
ros2 launch ur_llm_planner sim.launch.py              # UR3e (mặc định)
# ros2 launch ur_llm_planner sim.launch.py ur_type:=ur3
```

Đợi đến khi log hiện `Camera quan sat duoc:` (vị trí 5 khối) và `Skill server san sang`.

**Terminal 2** — LLM planner (chế độ nhập lệnh bằng bàn phím):

```bash
source ~/ur_ws/install/setup.bash
export NINEROUTER_API_KEY=<api_key>
ros2 run ur_llm_planner llm_planner_node.py --ros-args -p llm.model:=<ten_model>
```

Rồi gõ lệnh:

```
>>> Put the red cube in Zone B.
>>> Arrange all objects according to my student ID.
>>> Đưa khối màu tím vào vùng A.
```

### 3.1. Các cách gửi lệnh khác

```bash
# Gửi câu lệnh qua topic (planner đang chạy)
ros2 topic pub --once /nl_command std_msgs/msg/String "{data: 'Move the blue cube to zone C.'}"

# Chỉ sinh + kiểm tra kế hoạch, không di chuyển robot (dry run)
ros2 run ur_llm_planner llm_planner_node.py --ros-args -p llm.model:=<ten_model> -p execute:=false

# Kiểm thử Validator/Executor không qua LLM: gửi thẳng kế hoạch JSON
ros2 topic pub --once /plan_json std_msgs/msg/String \
  "{data: '{\"plan\": [{\"skill\": \"pick\", \"object\": \"red_cube\"}, {\"skill\": \"place\", \"object\": \"red_cube\", \"zone\": \"zone_b\"}, {\"skill\": \"home\"}]}'}"
#  -> he thong tu chen check_zone(zone_b) + don blue_cube ra temporary_position

# Gọi trực tiếp một skill
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: check_zone, zone: zone_b}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: detect_objects}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: pick, object: blue_cube}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: find_free_position}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: place, object: blue_cube, zone: temporary_position}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: home}"
```

### 3.2. Topic / Service

| Tên | Kiểu | Mô tả |
|---|---|---|
| `/camera/image`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Ảnh camera trên cao (bridge từ Gazebo) |
| `/perception/scene` | `ur_llm_planner/msg/SceneObservation` (latched) | Vị trí từng khối, khối nào đang nhìn thấy, vùng nào trống / có vật |
| `/perception/debug_image` | `sensor_msgs/Image` | Ảnh camera có vẽ vùng, contour, toạ độ khối (xem bằng `rqt_image_view`) |
| `/execute_skill` | `ur_llm_planner/srv/ExecuteSkill` | Thực thi một skill (skill_server) |
| `/scene_state` | `std_msgs/String` (JSON, latched) | Trạng thái tổng hợp: vật đang cầm, vị trí + nguồn (`camera`/`estimated`/`gripper`), `zone_state` |
| `/skill_server/held_object` | `std_msgs/String` (latched) | Vật đang cầm (`""` nếu không) |
| `/nl_command` | `std_msgs/String` | Câu lệnh ngôn ngữ tự nhiên |
| `/plan_json` | `std_msgs/String` | Kế hoạch JSON gửi thẳng (bỏ qua LLM, để kiểm thử) |
| `/llm_planner/plan` | `std_msgs/String` | Kế hoạch đã qua validator |
| `/llm_planner/result` | `std_msgs/String` (JSON) | Kết quả: `SUCCESS` / `FAILED` / `REJECTED` / `LLM_ERROR`, kết quả từng bước |
| `/skill_server/markers` | `visualization_msgs/MarkerArray` | Vùng A/B/C trên RViz (Add → By topic) |

## 4. Thay đổi môi trường

- **Thêm khối**: thêm model vào `worlds/pick_place.sdf` (màu bão hoà, khác hẳn các khối khác),
  thêm tên + `color` vào `config/scene.yaml`, thêm dải HSV cho màu mới vào
  `config/perception.yaml`, và thêm tên vào mặc định của `grasp_objects` trong
  `urdf/ur_robotiq.urdf.xacro` (để có DetachableJoint). Không cần khai báo vị trí khối.
- **Di chuyển khối ban đầu**: chỉ sửa `<pose>` trong `worlds/pick_place.sdf`; camera tự thấy.
- **Thêm / di chuyển vùng**: sửa `zone_names`/`zones` trong `scene.yaml` và model tương ứng trong
  world (tấm màu xám, độ bão hoà thấp).
- **Di chuyển camera**: chỉ sửa `<pose>` của model `overhead_camera` trong world; launch tự đọc.
- **Chỉnh màu**: xem `/perception/debug_image`; chỉnh `perception.hsv.<màu>` trong
  `config/perception.yaml` (thang OpenCV: H 0–179, S/V 0–255).

Các tham số chính của `skill_server`: `ee_link` (`grasp_tcp`), `approach_height` (0.12 m),
`velocity_scaling` (0.3), `acceleration_scaling` (0.3), `planning_time` (5 s), `planning_retries`
(3), `min_cartesian_fraction` (0.98), `home_joints` (`[3.14, -1.5708, 1.5708, -1.5708, -1.5708, 0]`
— cánh tay ở phía sau robot, không che camera), `perception_timeout` (3 s),
`free_space.*` (lưới ô trống cho `find_free_position`).

Tham số `llm_planner_node`: `auto_resolve` (true — hệ thống tự chèn `check_zone` + bước dọn
vùng), `max_replans` (1 — số lần lập lại kế hoạch khi `check_zone` thấy môi trường thay đổi),
`llm.max_repair_attempts` (2).

Tham số gripper: `gripper_open_position` (0.0), `gripper_grasp_position` (0.50 rad, khe 34 mm cho
khối 4 cm), `grasp_contact_tolerance` (0.02 rad), `gripper_motion_time` (1.0 s), `gripper_action`
(`/gripper_controller/follow_joint_trajectory`). Nếu đổi `cube_size` thì chọn lại
`gripper_grasp_position` sao cho khe hở nhỏ hơn cạnh khối khoảng 5–6 mm (xem bảng ở mục 1.4).

## 5. Kiểm thử

```bash
cd ~/ur_ws
colcon test --packages-select ur_llm_planner --event-handlers console_direct+
# hoặc chạy nhanh: python3 -m pytest src/ur_llm_planner/test -q
```

87 test, chạy không cần ROS hay Gazebo:

- `test/test_plan_validator.py` (42): skill/object/zone lạ, LLM gửi `joints`/`trajectory`/toạ
  độ, thiếu tham số, sai thứ tự, kế hoạch rỗng, trích JSON từ markdown, vùng bị chiếm,
  kế hoạch demo của đề, `temporary_position` cần `find_free_position` và chỉ dùng một lần, trạng
  thái dự kiến của `check_zone`, skill quan sát không phá thứ tự `move_above → close_gripper`,
  `place_on_table`/`zone_tmp` không còn tồn tại, kế hoạch MSSV, giới hạn 40 bước.
- `test/test_plan_resolver.py` (9): kế hoạch "ngây thơ" được bổ sung đúng như ví dụ của đề, vùng
  trống chỉ thêm `check_zone`, kế hoạch đầy đủ của LLM giữ nguyên, LLM tự dọn sang vùng khác,
  kiểm tra lại vùng sau khi đặt vật, 2 vùng bị chiếm, không có camera.
- `test/test_student.py` (19): bảng P của đề (red/yellow/blue), ví dụ 23020123 → P = 5, MSSV
  23020764 → P = 4, green/purple nằm ngoài vùng, MSSV không hợp lệ, nội dung prompt (trạng thái vùng, khối camera chưa thấy, vật đang cầm).
- `test/test_perception.py` (17): hình học camera (tâm ảnh, chiều trục, pixel ↔ world), dải màu
  đỏ vòng qua 0, tâm mặt trên bỏ mặt bên tối, loại blob bị che, không nhầm nắp khớp UR3e với
  `blue_cube`, gán vùng.

## 6. Xử lý lỗi thường gặp

- `LLM_ERROR: Khong ket noi duoc ...`: 9Router chưa chạy, hoặc sai `llm.base_url`.
- `LLM_ERROR: HTTP 401`: sai hoặc thiếu API key (`NINEROUTER_API_KEY`).
- `LLM_ERROR: Chua cau hinh llm.model`: chưa truyền `-p llm.model:=...`.
- `Service /execute_skill chua san sang`: terminal 1 chưa chạy xong. Đợi dòng `Skill server san sang`.
- `Chua co trang thai moi truong tu camera (/scene_state)`: skill_server hoặc perception_node
  chưa chạy. Kiểm tra `ros2 topic hz /camera/image` (~10 Hz) và `ros2 topic echo /perception/scene`.
- Không có ảnh trên `/camera/image`: thiếu plugin `ignition-gazebo-sensors-system` trong world,
  hoặc máy không render được OpenGL (WSL2 không có GPU). Xem log `ign gazebo`.
- `MOI TRUONG THAY DOI: ke hoach du kien ..., camera thay ...`: có khối được đặt vào / lấy khỏi
  vùng trong lúc robot đang chạy. Với câu lệnh ngôn ngữ tự nhiên, hệ thống tự lập lại kế hoạch.
- `OBJECT_NOT_VISIBLE` / log `[Camera] ... chua thay: <khối>`: khối bị che hoặc màu nằm ngoài dải
  HSV. Xem `/perception/debug_image` để chỉnh `config/perception.yaml`.
- `NO_OBJECT_TO_GRASP: Ngon kep dong het ma khong cham vat`: ngón không bị khối chặn lại (khối
  không nằm giữa 2 ngón). Kiểm tra vị trí camera báo so với thực tế.
- `package 'robotiq_description' not found`: chưa cài `ros-humble-robotiq-description`.
- `Action /gripper_controller/follow_joint_trajectory chua san sang`: `gripper_controller` chưa
  chạy. Kiểm tra bằng `ros2 control list_controllers`.

## 7. Hạn chế đã biết

- Khi gắp khối ở vùng gần trục x của robot (`y ≈ 0`), khối lệch khoảng 8–9 mm trong tay kẹp theo
  phương x; đặt vào vùng vẫn nằm gọn trong vùng (vùng rộng 80 mm), các vị trí khác lệch ~3 mm.
- Camera cố định trên cao: khối nằm ngay dưới gripper bị che. Hệ thống dùng vị trí ước lượng lúc
  thả cho tới khi robot rời đi và camera nhìn lại được.
- Nhận dạng dựa trên màu: mỗi khối phải có một màu riêng.
