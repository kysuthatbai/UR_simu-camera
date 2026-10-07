import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.prompt import build_system_prompt, build_user_prompt  # noqa: E402
from llm_planner.student import make_student_task  # noqa: E402

R, Y, B, G, P = "red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"
OBJECTS = {R: "red", Y: "yellow", B: "blue", G: "green", P: "purple"}
ZONES = {"zone_a": "A", "zone_b": "B", "zone_c": "C"}


@pytest.mark.parametrize("student_id, p, a, b, c", [
    # Bang quy uoc cua de bai: chi dung red / yellow / blue
    ("23020100", 0, R, Y, B),   # 00
    ("23020160", 0, R, Y, B),   # 60
    ("23020125", 1, R, B, Y),   # 25
    ("23020150", 2, Y, R, B),   # 50
    ("23020115", 3, Y, B, R),   # 15
    ("23020140", 4, B, R, Y),   # 40
    ("23020105", 5, B, Y, R),   # 05
    ("23020101", 1, R, B, Y),   # 01
    ("23020103", 3, Y, B, R),   # 03
])
def test_permutation_table(student_id, p, a, b, c):
    t = make_student_task("SV", student_id)
    assert t.p == p
    assert t.targets == {"zone_a": a, "zone_b": b, "zone_c": c}
    assert set(t.others) == {G, P}


def test_assignment_example():
    # Vi du trong de bai: 23020123 -> 23 mod 6 = 5 -> A = Blue, B = Yellow, C = Red
    t = make_student_task("Nguyen Van An", "23020123")
    assert t.formula() == "P = 23 mod 6 = 5"
    assert t.targets == {"zone_a": B, "zone_b": Y, "zone_c": R}


def test_real_student_id():
    t = make_student_task("Vu Ngoc Son", "23020764")   # 64 mod 6 = 4
    assert t.formula() == "P = 64 mod 6 = 4"
    assert t.assignment_lines() == ["zone_a <- blue_cube", "zone_b <- red_cube",
                                    "zone_c <- yellow_cube"]
    assert t.others == (G, P)


def test_integer_student_id_accepted():
    # ROS doc "-p student.id:=23020764" thanh so nguyen
    assert make_student_task("SV", 23020764).targets["zone_a"] == B


@pytest.mark.parametrize("bad", ["", "7", "23A20123", "abc"])
def test_invalid_student_id_rejected(bad):
    with pytest.raises(ValueError):
        make_student_task("SV", bad)


def test_system_prompt_contains_personal_task_and_table_skill():
    t = make_student_task("Vu Ngoc Son", "23020764")
    prompt = build_system_prompt(OBJECTS, ZONES, t)
    assert "MSSV 23020764" in prompt
    assert "zone_a <- blue_cube" in prompt
    assert "zone_b <- red_cube" in prompt
    assert "zone_c <- yellow_cube" in prompt
    assert "green_cube, purple_cube" in prompt
    assert "mod 5" not in prompt
    assert "find_free_position" in prompt and "temporary_position" in prompt
    assert "place_on_table" not in prompt
    assert "zone_tmp" not in prompt


def test_user_prompt_summarizes_zones_without_coordinates():
    state = {"held_object": None,
             "objects": {R: {"position": [0.2, 0.2, 0.02], "in_zone": "zone_b",
                             "source": "camera"},
                         G: {"position": [0.1, 0.28, 0.02], "in_zone": "zone_c",
                             "source": "camera"},
                         Y: {"position": [0.25, -0.17, 0.02], "in_zone": None,
                             "source": "camera"},
                         B: {"position": None, "in_zone": None, "source": "unknown"}},
             "zones": ["zone_a", "zone_b", "zone_c"]}
    text = build_user_prompt("Arrange all objects according to my student ID.", state)
    assert "zone_a: trong" in text
    assert "zone_b: dang co red_cube" in text
    assert "zone_c: dang co green_cube" in text
    assert "yellow_cube: tren ban, ngoai moi vung" in text
    assert "blue_cube: camera chua thay" in text
    assert "0.2" not in text


def test_held_object_not_counted_in_zone():
    state = {"held_object": R,
             "objects": {R: {"position": [0.2, 0.2, 0.15], "in_zone": None,
                             "source": "gripper"}},
             "zones": ["zone_a", "zone_b", "zone_c"]}
    text = build_user_prompt("x", state)
    assert "Gripper dang cam: red_cube" in text
    assert "zone_b: trong" in text
