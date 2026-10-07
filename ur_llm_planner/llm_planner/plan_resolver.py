"""Plan Resolver: logic xu ly vung bi chiem do HE THONG thuc hien, khong phu thuoc vao LLM.

LLM duoc huong dan tu chen cac buoc kiem tra / don vung, nhung neu LLM bo sot thi resolver bo
sung. Voi moi cap pick(X) ... place(X, Z) (Z la mot vung):

  1. Chen check_zone(Z) truoc pick(X) neu Z chua duoc kiem tra (hoac da co vat duoc dat vao Z
     sau lan kiem tra truoc) -> camera xac nhan trang thai Z ngay truoc khi thao tac.
  2. Neu (theo trang thai camera + mo phong cac buoc truoc) Z dang co vat Y khac X, chen:
         pick(Y) -> find_free_position() -> place(Y, temporary_position)
     truoc pick(X).

Resolver chi them buoc, khong xoa / doi buoc cua LLM. Cac buoc them vao co inserted = True.
Ke hoach sau khi bo sung VAN phai qua Plan Validator.
"""

from typing import List, Mapping, Optional, Sequence, Tuple

from llm_planner.plan_validator import OBSERVATION_SKILLS, TEMP_POSITION, Step


def _destination(steps: Sequence[Step], i: int) -> Optional[str]:
    """Dich cua vat duoc pick o buoc i: zone cua place(vat, zone) tiep theo (None neu khac)."""
    obj = steps[i].object
    for s in steps[i + 1:]:
        if s.skill in OBSERVATION_SKILLS or s.skill == "home":
            continue
        if s.skill == "place" and s.object == obj:
            return s.zone
        return None
    return None


def resolve_plan(steps: Sequence[Step], zones: Sequence[str], held: Optional[str] = None,
                 locations: Optional[Mapping[str, Optional[str]]] = None
                 ) -> Tuple[List[Step], List[str]]:
    """Tra ve (ke hoach da bo sung, danh sach ghi chu ve cac buoc da chen)."""
    zones = set(zones)
    loc = dict(locations) if locations is not None else None
    if loc is not None and held:
        loc[held] = None
    out: List[Step] = []
    notes: List[str] = []
    checked = set()   # vung da check_zone va chua co vat nao duoc dat vao sau do
    holding = held
    above: Optional[str] = None
    at_zone: Optional[str] = None

    for i, s in enumerate(steps):
        block_start = (s.skill == "pick" and not holding) or \
                      (s.skill == "place" and holding == s.object and
                       not (out and out[-1].skill == "pick"))
        dest = _destination(steps, i) if s.skill == "pick" else s.zone
        if block_start and dest in zones:
            if dest not in checked:
                out.append(Step("check_zone", zone=dest, inserted=True))
                checked.add(dest)
                notes.append(f"chen check_zone({dest}) truoc khi dat '{s.object}' vao {dest}")
            other = next((o for o, z in loc.items() if z == dest and o != s.object), None) \
                if loc is not None else None
            if other and s.skill == "pick":
                out += [Step("pick", object=other, inserted=True),
                        Step("find_free_position", inserted=True),
                        Step("place", object=other, zone=TEMP_POSITION, inserted=True)]
                loc[other] = None
                notes.append(f"{dest} dang co '{other}' (camera) -> chen pick({other}) -> "
                             f"find_free_position() -> place({other}, {TEMP_POSITION})")
        out.append(s)

        # Mo phong buoc cua LLM de biet trang thai cho cac buoc sau
        if s.skill == "check_zone":
            checked.add(s.zone)
        elif s.skill == "pick":
            holding = s.object
            if loc is not None:
                loc[s.object] = None
            above = at_zone = None
        elif s.skill == "place":
            if loc is not None:
                loc[s.object] = None if s.zone == TEMP_POSITION else s.zone
            checked.discard(s.zone)
            holding = above = at_zone = None
        elif s.skill == "move_above":
            above, at_zone = s.object, None
        elif s.skill == "move_to_zone":
            above, at_zone = None, s.zone
        elif s.skill == "close_gripper":
            holding = above
            if loc is not None and holding:
                loc[holding] = None
            above = at_zone = None
        elif s.skill == "open_gripper":
            if holding and at_zone:
                if loc is not None:
                    loc[holding] = None if at_zone == TEMP_POSITION else at_zone
                checked.discard(at_zone)
            holding = above = at_zone = None
        elif s.skill == "home":
            above = at_zone = None
    return out, notes
