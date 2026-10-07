"""Ca nhan hoa nhiem vu theo ma so sinh vien (MSSV).

Voi XX la 2 chu so cuoi cua MSSV, P = XX mod 6 quyet dinh khoi nao dat vao zone A / B / C
(theo de bai):

    P = 0 -> A = Red,    B = Yellow, C = Blue
    P = 1 -> A = Red,    B = Blue,   C = Yellow
    P = 2 -> A = Yellow, B = Red,    C = Blue
    P = 3 -> A = Yellow, B = Blue,   C = Red
    P = 4 -> A = Blue,   B = Red,    C = Yellow
    P = 5 -> A = Blue,   B = Yellow, C = Red

Vi du: MSSV 23020123 -> 23 mod 6 = 5 -> A = Blue, B = Yellow, C = Red.

Cac khoi con lai trong scene (green, purple) khong thuoc nhiem vu -> phai nam NGOAI ca 3 vung
(tren ban) de 3 vung chi chua dung 3 khoi duoc giao.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple

TASK_ZONES: Tuple[str, str, str] = ("zone_a", "zone_b", "zone_c")
# 3 khoi cua nhiem vu ca nhan, theo thu tu dung trong bang P (c0, c1, c2)
TASK_BLOCKS: Tuple[str, str, str] = ("red_cube", "yellow_cube", "blue_cube")
# Cac khoi khac trong scene, khong duoc giao vung nao
OTHER_BLOCKS: Tuple[str, ...] = ("green_cube", "purple_cube")

# P -> chi so (trong TASK_BLOCKS) cua khoi o zone_a, zone_b, zone_c
PERMUTATIONS: Dict[int, Tuple[int, int, int]] = {
    0: (0, 1, 2),
    1: (0, 2, 1),
    2: (1, 0, 2),
    3: (1, 2, 0),
    4: (2, 0, 1),
    5: (2, 1, 0),
}


@dataclass(frozen=True)
class StudentTask:
    name: str
    student_id: str
    xx: int
    p: int
    # zone -> vat phai dat vao zone do
    targets: Dict[str, str]
    # cac khoi khong duoc giao vung nao -> phai nam ngoai 3 vung
    others: Tuple[str, ...]

    def formula(self) -> str:
        return f"P = {self.xx:02d} mod 6 = {self.p}"

    def assignment_lines(self) -> List[str]:
        return [f"{zone} <- {obj}" for zone, obj in self.targets.items()]

    def others_line(self) -> str:
        return f"ngoai cac vung (tren ban): {', '.join(self.others)}"


def make_student_task(name: str, student_id) -> StudentTask:
    # MSSV co the duoc ROS doc thanh so nguyen (vd: -p student.id:=23020123) -> ep ve chuoi
    sid = str(student_id).strip()
    if not sid.isdigit() or len(sid) < 2:
        raise ValueError(f"MSSV khong hop le: '{sid}' (phai la chuoi chu so, it nhat 2 chu so)")
    xx = int(sid[-2:])
    p = xx % 6
    targets = {zone: TASK_BLOCKS[i] for zone, i in zip(TASK_ZONES, PERMUTATIONS[p])}
    return StudentTask(name=str(name).strip(), student_id=sid, xx=xx, p=p,
                       targets=targets, others=OTHER_BLOCKS)
