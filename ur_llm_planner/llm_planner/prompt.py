"""Xay dung prompt cho LLM Task Planner.

Prompt chi mo ta: danh sach skill, danh sach vat / vung hop le va trang thai hien tai (do camera
quan sat). LLM chi duoc tra ve JSON {"plan": [...]} gom cac skill -> khong co toa do, khong co
khop.
"""

import json
from typing import Dict, List, Optional

from llm_planner.student import StudentTask

SKILL_DOCS = [
    # Quan sat bang camera - robot KHONG di chuyen
    ("detect_objects", "{}", "[camera] liet ke moi vat va vung chua vat"),
    ("check_zone", '{"zone": <zone>}',
     "[camera] kiem tra vung dang trong hay dang co vat nao. Dung TRUOC khi dat vat vao vung"),
    ("find_object", '{"object": <object>}', "[camera] tim vi tri cua mot vat"),
    ("find_free_position", "{}",
     "[camera] tim mot vi tri trong tren ban (ngoai moi vung, robot voi toi duoc) va dat ten no "
     "la temporary_position. Moi lan can mot vi tri tam moi thi goi lai"),
    # Chuyen dong
    ("home", "{}", "dua robot ve tu the nghi an toan"),
    ("pick", '{"object": <object>}',
     "di toi phia tren vat, ha xuong, kep vat bang 2 ngon gripper va nang len"),
    ("place", '{"object": <object>, "zone": <zone | "temporary_position">}',
     "mang vat DANG CAM toi vung (hoac toi temporary_position vua tim), ha xuong, tha vat"),
    ("move_above", '{"object": <object>}', "di toi phia tren mot vat (khong gap)"),
    ("move_to_zone", '{"zone": <zone | "temporary_position">}',
     "di toi phia tren mot vung (khong tha)"),
    ("close_gripper", "{}", "ha xuong va kep vat nam ngay duoi gripper (dung sau move_above)"),
    ("open_gripper", "{}", "ha vat dang cam xuong va mo ngon kep (dung sau move_to_zone)"),
]

EXAMPLES = [
    ("Trang thai hien tai:\n- Gripper dang cam: khong co gi\n- zone_a: trong\n"
     "- zone_b: trong\n- zone_c: trong\nCau lenh: Put the red cube in zone A.",
     {"plan": [{"skill": "check_zone", "zone": "zone_a"},
               {"skill": "pick", "object": "red_cube"},
               {"skill": "place", "object": "red_cube", "zone": "zone_a"},
               {"skill": "home"}]}),
    ("Trang thai hien tai:\n- Gripper dang cam: khong co gi\n- zone_a: trong\n"
     "- zone_b: dang co blue_cube\n- zone_c: trong\nCau lenh: Put the red cube in Zone B.",
     {"plan": [{"skill": "check_zone", "zone": "zone_b"},
               {"skill": "pick", "object": "blue_cube"},
               {"skill": "find_free_position"},
               {"skill": "place", "object": "blue_cube", "zone": "temporary_position"},
               {"skill": "pick", "object": "red_cube"},
               {"skill": "place", "object": "red_cube", "zone": "zone_b"},
               {"skill": "home"}]}),
    ("Cau lenh: Hãy nấu cho tôi một bát phở.",
     {"plan": [], "reason": "Yeu cau khong lien quan toi cac skill cua robot"}),
]


def build_student_section(task: Optional[StudentTask]) -> str:
    if task is None:
        return ""
    targets = "\n".join(f"- {line}" for line in task.assignment_lines())
    return f"""
NHIEM VU CA NHAN CUA SINH VIEN (dung khi nguoi dung nhac toi "student ID", "MSSV",
"ma sinh vien", "my task", "nhiem vu ca nhan"...):
Sinh vien {task.name}, MSSV {task.student_id}, {task.formula()}. Vi tri dich cua moi vat:
{targets}
- {task.others_line()} -> neu dang nam trong mot vung thi pick, find_free_position roi
  place(<vat>, temporary_position).
"""


def build_system_prompt(objects: Dict[str, str], zones: Dict[str, str],
                        student: Optional[StudentTask] = None) -> str:
    skills = "\n".join(f"- {name}: args {args} -> {doc}" for name, args, doc in SKILL_DOCS)
    objs = "\n".join(f"- {name}: {desc}" for name, desc in objects.items())
    zns = "\n".join(f"- {name}: {desc}" for name, desc in zones.items())
    examples = "\n\n".join(
        f"User: {cmd}\nAssistant: {json.dumps(plan, ensure_ascii=False)}" for cmd, plan in EXAMPLES)
    return f"""Ban la bo lap ke hoach nhiem vu (task planner) cho canh tay robot UR3 trong mo phong.
Nhiem vu: hieu cau lenh ngon ngu tu nhien (tieng Viet hoac tieng Anh) va chuyen no thanh mot
chuoi cac ROBOT SKILL co san. Ban KHONG dieu khien robot truc tiep.

SKILL DUOC PHEP (chi duoc dung dung cac ten nay; [camera] = chi quan sat, robot dung yen):
{skills}

VAT THE HOP LE (dung dung ten ben trai):
{objs}

VUNG DAT VAT HOP LE (dung dung ten ben trai):
{zns}
{build_student_section(student)}
QUY TAC:
1. Chi tra ve DUY NHAT mot JSON object dang {{"plan": [{{"skill": ..., ...}}, ...]}}. Khong giai thich,
   khong markdown.
2. Moi buoc chi co khoa "skill" va dung cac tham so cua skill do ("object", "zone"). Tuyet doi
   KHONG sinh toa do, goc khop (joint), quy dao, van toc hay bat ky gia tri so nao.
3. Anh xa mo ta cua nguoi dung (mau sac, "vat", "khoi", "cube", "o", "vung", "zone"...) sang dung ten
   object/zone o tren. Uu tien dung pick va place; pick phai dung truoc place cua cung vat.
4. Robot chi cam duoc 1 vat mot luc. Voi nhieu vat, lap lai pick -> place cho tung vat.
5. Ket thuc ke hoach bang {{"skill": "home"}} tru khi nguoi dung yeu cau khac.
6. Neu yeu cau khong the thuc hien bang cac skill tren, hoac nhac toi vat/vung khong ton tai,
   tra ve {{"plan": [], "reason": "<ly do ngan gon>"}}.
7. "Trang thai hien tai" do CAMERA quan sat. Moi vung chi chua DUNG 1 vat. Truoc khi dat vat
   vao vung Z, goi check_zone(Z). KHONG place vao vung dang co vat khac. Neu Z dang bi vat X
   chiem: chuyen X toi vung dich cua X neu vung do dang trong; neu khong thi
   pick(X) -> find_free_position() -> place(X, temporary_position). Sau do moi pick va place
   vat can dat vao Z. Cap nhat trang thai cac vung sau moi buoc khi lap ke hoach.
8. Vat da nam dung vung dich thi khong di chuyen. Neu khong can di chuyen vat nao,
   tra ve {{"plan": [{{"skill": "home"}}]}}.
9. Vat "camera chua thay" thi khong the gap: tra ve plan rong kem ly do neu cau lenh can vat do.

VI DU:
{examples}"""


def describe_scene(scene_state: dict) -> str:
    """Tom tat /scene_state cho LLM: vat dang cam, vat nao o vung nao, vung nao trong.

    Khong dua toa do vao prompt de LLM khong co gia tri so nao de "bat chuoc".
    """
    objects = scene_state.get("objects") or {}
    zones = scene_state.get("zones") or []
    held = scene_state.get("held_object")
    lines = [f"- Gripper dang cam: {held or 'khong co gi'}"]
    occupant: Dict[str, str] = {}
    for name, info in objects.items():
        if name == held:
            continue
        info = info or {}
        zone = info.get("in_zone")
        if zone and zone not in occupant:
            occupant[zone] = name
    for zone in zones:
        lines.append(f"- {zone}: " + (f"dang co {occupant[zone]}" if zone in occupant else "trong"))
    for name, info in objects.items():
        if name == held:
            continue
        info = info or {}
        if info.get("position") is None and info.get("source") == "unknown":
            lines.append(f"- {name}: camera chua thay")
        elif not info.get("in_zone"):
            lines.append(f"- {name}: tren ban, ngoai moi vung")
    return "\n".join(lines)


def build_user_prompt(command: str, scene_state: Optional[dict]) -> str:
    if scene_state:
        return f"Trang thai hien tai:\n{describe_scene(scene_state)}\nCau lenh: {command}"
    return f"Cau lenh: {command}"


def build_messages(command: str, objects: Dict[str, str], zones: Dict[str, str],
                   scene_state: Optional[dict] = None,
                   student: Optional[StudentTask] = None) -> List[Dict[str, str]]:
    return [
        {"role": "system", "content": build_system_prompt(objects, zones, student)},
        {"role": "user", "content": build_user_prompt(command, scene_state)},
    ]


def build_repair_message(errors: List[str]) -> Dict[str, str]:
    """Tin nhan gui lai cho LLM khi ke hoach bi Plan Validator tu choi."""
    joined = "\n".join(f"- {e}" for e in errors)
    return {"role": "user",
            "content": f"Ke hoach tren KHONG hop le:\n{joined}\n"
                       "Hay tra ve lai JSON ke hoach da sua, tuan thu dung cac quy tac."}
