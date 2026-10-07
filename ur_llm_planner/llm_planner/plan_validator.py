"""Plan Validator: kiem tra ke hoach JSON do LLM sinh ra TRUOC khi gui toi robot.

Nguyen tac: whitelist. Chi cac skill trong SKILLS, object trong danh sach object va zone
trong danh sach zone moi duoc chap nhan. Bat ky truong nao khac (vd: joint, trajectory,
toa do x/y/z...) deu bi tu choi -> LLM khong the dieu khien truc tiep khop cua robot.
Neu co BAT KY loi nao, ca ke hoach bi tu choi (khong thuc thi mot phan).

Khi biet vat nao dang nam o vung nao (locations, lay tu /scene_state - do camera xac dinh),
validator con mo phong trang thai cac vung: moi vung chi chua 1 vat, khong duoc dat vat vao vung
dang co vat khac. Vat dang chiem cho duoc chuyen toi vung dich cua no, hoac toi mot vi tri trong
tren ban: find_free_position() -> place(vat, temporary_position).

Voi moi buoc check_zone(Z), validator ghi lai vat ma ke hoach DU KIEN dang nam o Z (Step.expect).
Luc thuc thi, llm_planner_node so sanh voi ket qua camera; khac nhau -> dung lai, lap ke hoach moi.
"""

import json
import re
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

# Dich tam: o trong tren ban do find_free_position tim duoc (skill_server tu tinh toa do).
TEMP_POSITION = "temporary_position"

# skill -> danh sach tham so bat buoc (va cung la tham so duy nhat duoc phep)
SKILLS: Dict[str, Tuple[str, ...]] = {
    # Quan sat bang camera - khong di chuyen robot
    "detect_objects": (),
    "check_zone": ("zone",),
    "find_object": ("object",),
    "find_free_position": (),
    # Chuyen dong
    "home": (),
    "pick": ("object",),
    "place": ("object", "zone"),
    "move_above": ("object",),
    "move_to_zone": ("zone",),
    "open_gripper": (),
    "close_gripper": (),
}
OBSERVATION_SKILLS = frozenset({"detect_objects", "check_zone", "find_object",
                                "find_free_position"})
# Skill duoc phep nhan temporary_position lam "zone"
TEMP_TARGET_SKILLS = frozenset({"place", "move_to_zone"})

# 5 khoi x (pick + place) + check_zone + cac buoc don vung bi chiem + home
MAX_STEPS = 40


@dataclass
class Step:
    skill: str
    object: str = ""
    zone: str = ""
    # check_zone: vat du kien dang nam o vung ("" = trong); None = khong biet (khong kiem tra)
    expect: Optional[str] = field(default=None, compare=False)
    # True: buoc do he thong tu chen (plan_resolver), khong phai do LLM sinh ra
    inserted: bool = field(default=False, compare=False)

    def to_dict(self) -> dict:
        d = {"skill": self.skill}
        if self.object:
            d["object"] = self.object
        if self.zone:
            d["zone"] = self.zone
        return d

    def __str__(self) -> str:
        args = [a for a in (self.object, self.zone) if a]
        return f"{self.skill}({', '.join(args)})"


@dataclass
class ValidationResult:
    ok: bool
    steps: List[Step] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    # Ly do LLM tu choi (khi LLM tra ve plan rong kem "reason")
    reason: str = ""


def extract_json(text: str) -> dict:
    """Lay doi tuong JSON tu cau tra loi cua LLM (chap nhan ```json ... ``` hoac text thua)."""
    if text is None:
        raise ValueError("LLM khong tra ve noi dung")
    text = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL)
    if fence:
        text = fence.group(1).strip()
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise ValueError("Khong tim thay JSON trong cau tra loi cua LLM")
        try:
            data = json.loads(text[start:end + 1])
        except json.JSONDecodeError as e:
            raise ValueError(f"JSON khong hop le: {e}") from e
    if not isinstance(data, dict):
        raise ValueError("JSON goc phai la object dang {\"plan\": [...]}")
    return data


def _norm(value) -> Optional[str]:
    if not isinstance(value, str):
        return None
    return value.strip().lower()


def validate_plan(
    data: dict,
    objects: Sequence[str],
    zones: Sequence[str],
    held_object: Optional[str] = None,
    locations: Optional[Mapping[str, Optional[str]]] = None,
) -> ValidationResult:
    """Kiem tra cu phap + ngu nghia cua ke hoach.

    held_object: vat robot dang cam truoc khi chay ke hoach (lay tu /scene_state).
    locations: vat -> vung dang chua vat (None neu khong nam trong vung nao). Neu None thi bo
        qua kiem tra vung bi chiem.
    """
    errors: List[str] = []
    steps: List[Step] = []
    objects = set(objects)
    zones = set(zones)

    if not isinstance(data, dict):
        return ValidationResult(False, errors=["Ke hoach phai la JSON object"])

    unknown_top = set(data) - {"plan", "reason"}
    if unknown_top:
        errors.append(f"Truong khong hop le o muc goc: {sorted(unknown_top)}")

    plan = data.get("plan")
    reason = data.get("reason", "") if isinstance(data.get("reason", ""), str) else ""
    if not isinstance(plan, list):
        return ValidationResult(False, errors=errors + ["Thieu truong 'plan' dang danh sach"])
    if len(plan) == 0:
        msg = "Ke hoach rong - LLM khong chon duoc skill nao"
        if reason:
            msg += f" (ly do: {reason})"
        return ValidationResult(False, errors=errors + [msg], reason=reason)
    if len(plan) > MAX_STEPS:
        errors.append(f"Ke hoach qua dai ({len(plan)} buoc > {MAX_STEPS})")

    for i, raw in enumerate(plan, start=1):
        where = f"Buoc {i}"
        if not isinstance(raw, dict):
            errors.append(f"{where}: moi buoc phai la object JSON")
            continue
        skill = _norm(raw.get("skill"))
        if skill is None:
            errors.append(f"{where}: thieu 'skill' hoac 'skill' khong phai chuoi")
            continue
        if skill not in SKILLS:
            errors.append(f"{where}: skill '{raw.get('skill')}' khong nam trong danh sach cho phep "
                          f"{sorted(SKILLS)}")
            continue
        required = SKILLS[skill]
        extra = set(raw) - {"skill"} - set(required)
        if extra:
            errors.append(f"{where} ({skill}): tham so khong cho phep {sorted(extra)}")
        step = Step(skill=skill)
        for arg in required:
            value = _norm(raw.get(arg))
            if not value:
                errors.append(f"{where} ({skill}): thieu tham so '{arg}'")
                continue
            if arg == "object" and value not in objects:
                errors.append(f"{where} ({skill}): object '{raw.get(arg)}' khong ton tai "
                              f"(hop le: {sorted(objects)})")
                continue
            if arg == "zone" and value not in zones and not (
                    value == TEMP_POSITION and skill in TEMP_TARGET_SKILLS):
                allowed = sorted(zones) + ([TEMP_POSITION] if skill in TEMP_TARGET_SKILLS else [])
                errors.append(f"{where} ({skill}): zone '{raw.get(arg)}' khong hop le "
                              f"(hop le: {allowed})")
                continue
            setattr(step, arg, value)
        steps.append(step)

    if not errors:
        errors.extend(_check_sequence(steps, held_object, locations))

    return ValidationResult(ok=not errors, steps=steps if not errors else [], errors=errors,
                            reason=reason)


def _check_sequence(steps: List[Step], held: Optional[str],
                    locations: Optional[Mapping[str, Optional[str]]] = None) -> List[str]:
    """Mo phong trang thai gripper (va cac vung) de phat hien thu tu skill vo ly.

    Vd: place khi chua pick, pick khi dang cam vat khac, place vao vung dang co vat khac,
    place(..., temporary_position) khi chua find_free_position.
    locations: vat -> vung (None = nam tren ban, ngoai moi vung). Ghi Step.expect cho check_zone.
    """
    errors = []
    loc = dict(locations) if locations is not None else None
    if loc is not None and held:
        loc[held] = None
    above: Optional[str] = None    # vat ma gripper dang o ngay phia tren
    at_zone: Optional[str] = None  # vung (hoac temporary_position) ma gripper dang o phia tren
    temp_ready = False             # da co temporary_position chua dung

    def occupant(zone: str, exclude: Optional[str] = None) -> Optional[str]:
        return next((o for o, z in loc.items() if z == zone and o != exclude), None)

    def check_free(where: str, zone: str, obj: str):
        if loc is None:
            return
        other = occupant(zone, obj)
        if other:
            errors.append(f"{where}: {zone} dang co '{other}' - phai chuyen '{other}' sang vung "
                          f"dich cua no hoac find_free_position() -> place('{other}', "
                          f"{TEMP_POSITION}) truoc khi dat '{obj}'")

    def need_temp(where: str):
        if not temp_ready:
            errors.append(f"{where}: chua co {TEMP_POSITION} - phai goi find_free_position() "
                          "truoc")

    for i, s in enumerate(steps, start=1):
        where = f"Buoc {i} ({s})"
        if s.skill in OBSERVATION_SKILLS:
            # Chi quan sat, robot khong di chuyen -> giu nguyen above / at_zone
            if s.skill == "check_zone" and loc is not None:
                s.expect = occupant(s.zone) or ""
            elif s.skill == "find_free_position":
                temp_ready = True
        elif s.skill == "pick":
            if held:
                errors.append(f"{where}: dang cam '{held}', phai place truoc khi pick vat khac")
            held = s.object
            if loc is not None:
                loc[s.object] = None
            above = at_zone = None
        elif s.skill == "place":
            if held != s.object:
                errors.append(f"{where}: gripper khong cam '{s.object}' "
                              f"(dang cam: {held or 'khong co gi'})")
            elif s.zone == TEMP_POSITION:
                need_temp(where)
            else:
                check_free(where, s.zone, s.object)
            if loc is not None:
                loc[s.object] = None if s.zone == TEMP_POSITION else s.zone
            if s.zone == TEMP_POSITION:
                temp_ready = False
            held = above = at_zone = None
        elif s.skill == "move_above":
            if held == s.object:
                errors.append(f"{where}: khong the di toi phia tren vat dang cam")
            above, at_zone = s.object, None
        elif s.skill == "move_to_zone":
            if s.zone == TEMP_POSITION:
                need_temp(where)
            above, at_zone = None, s.zone
        elif s.skill == "close_gripper":
            if held:
                errors.append(f"{where}: gripper dang cam '{held}'")
            elif not above:
                errors.append(f"{where}: close_gripper can move_above(object) ngay truoc do")
            held = above
            if loc is not None and held:
                loc[held] = None
            above = at_zone = None
        elif s.skill == "open_gripper":
            if held:
                if not at_zone:
                    errors.append(f"{where}: dang cam '{held}', open_gripper can "
                                  "move_to_zone(zone) ngay truoc do")
                elif at_zone == TEMP_POSITION:
                    if loc is not None:
                        loc[held] = None
                    temp_ready = False
                else:
                    check_free(where, at_zone, held)
                    if loc is not None:
                        loc[held] = at_zone
            held = above = at_zone = None
        else:  # home
            above = at_zone = None
    return errors
