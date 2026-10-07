import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.plan_validator import extract_json, validate_plan  # noqa: E402

OBJECTS = ["red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"]
ZONES = ["zone_a", "zone_b", "zone_c"]


def check(plan, held=None):
    return validate_plan(plan, OBJECTS, ZONES, held)


def test_valid_pick_place_home():
    r = check({"plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]})
    assert r.ok, r.errors
    assert [str(s) for s in r.steps] == ["pick(red_cube)", "place(red_cube, zone_b)", "home()"]


def test_valid_low_level_skills():
    r = check({"plan": [
        {"skill": "move_above", "object": "blue_cube"},
        {"skill": "close_gripper"},
        {"skill": "move_to_zone", "zone": "zone_c"},
        {"skill": "open_gripper"},
        {"skill": "home"},
    ]})
    assert r.ok, r.errors


def test_case_and_whitespace_normalized():
    r = check({"plan": [{"skill": " PICK ", "object": "Red_Cube"},
                        {"skill": "place", "object": "red_cube", "zone": "ZONE_A"}]})
    assert r.ok, r.errors
    assert r.steps[1].zone == "zone_a"


def test_unknown_skill_rejected():
    r = check({"plan": [{"skill": "fly", "object": "red_cube"}]})
    assert not r.ok
    assert "khong nam trong danh sach cho phep" in r.errors[0]


def test_unknown_object_rejected():
    r = check({"plan": [{"skill": "pick", "object": "orange_cube"}]})
    assert not r.ok
    assert "orange_cube" in r.errors[0]


def test_unknown_zone_rejected():
    r = check({"plan": [{"skill": "pick", "object": "red_cube"},
                        {"skill": "place", "object": "red_cube", "zone": "zone_d"}]})
    assert not r.ok
    assert "zone_d" in r.errors[0]


def test_joint_values_rejected():
    r = check({"plan": [{"skill": "home", "joints": [0, -1.57, 0, 0, 0, 0]}]})
    assert not r.ok
    assert "joints" in r.errors[0]


def test_coordinates_in_place_rejected():
    r = check({"plan": [{"skill": "pick", "object": "red_cube"},
                        {"skill": "place", "object": "red_cube", "zone": "zone_a",
                         "position": [0.3, 0.1, 0.0]}]})
    assert not r.ok


def test_trajectory_top_level_rejected():
    r = check({"plan": [{"skill": "home"}], "trajectory": [[0, 0, 0, 0, 0, 0]]})
    assert not r.ok


def test_missing_argument_rejected():
    r = check({"plan": [{"skill": "place", "object": "red_cube"}]})
    assert not r.ok
    assert "zone" in r.errors[0]


def test_place_without_pick_rejected():
    r = check({"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_a"}]})
    assert not r.ok
    assert "khong cam" in r.errors[0]


def test_place_allowed_when_already_holding():
    r = check({"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_a"}]},
              held="red_cube")
    assert r.ok, r.errors


def test_double_pick_rejected():
    r = check({"plan": [{"skill": "pick", "object": "red_cube"},
                        {"skill": "pick", "object": "blue_cube"}]})
    assert not r.ok


def test_close_gripper_needs_move_above():
    r = check({"plan": [{"skill": "close_gripper"}]})
    assert not r.ok


def test_empty_plan_with_reason_rejected():
    r = check({"plan": [], "reason": "khong lien quan"})
    assert not r.ok
    assert r.reason == "khong lien quan"


def test_plan_not_list_rejected():
    assert not check({"plan": "pick red"}).ok
    assert not check({"steps": []}).ok


def test_whole_plan_rejected_if_any_step_invalid():
    r = check({"plan": [{"skill": "pick", "object": "red_cube"},
                        {"skill": "place", "object": "red_cube", "zone": "zone_x"},
                        {"skill": "home"}]})
    assert not r.ok
    assert r.steps == []


def test_extract_json_from_markdown():
    text = 'Day la ke hoach:\n```json\n{"plan": [{"skill": "home"}]}\n```'
    assert extract_json(text) == {"plan": [{"skill": "home"}]}


def test_extract_json_with_surrounding_text():
    assert extract_json('Sure! {"plan": []} ok') == {"plan": []}


def test_extract_json_invalid():
    import pytest
    with pytest.raises(ValueError):
        extract_json("khong co json")


# ---------------------------------------------------------------- vung bi chiem (locations)
INITIAL = {o: None for o in OBJECTS}


def check_loc(plan, locations, held=None):
    return validate_plan(plan, OBJECTS, ZONES, held, locations)


def test_place_into_occupied_zone_rejected():
    r = check_loc({"plan": [{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"}]},
                  {**INITIAL, "blue_cube": "zone_a"})
    assert not r.ok
    assert "blue_cube" in r.errors[0]


def test_place_after_occupied_zone_cleared_ok():
    r = check_loc({"plan": [{"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": "zone_c"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
                            {"skill": "home"}]},
                  {**INITIAL, "blue_cube": "zone_a"})
    assert r.ok, r.errors


def test_swap_through_table_ok():
    r = check_loc({"plan": [{"skill": "pick", "object": "red_cube"},
                            {"skill": "find_free_position"},
                            {"skill": "place", "object": "red_cube", "zone": "temporary_position"},
                            {"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                            {"skill": "home"}]},
                  {**INITIAL, "red_cube": "zone_a", "blue_cube": "zone_b"})
    assert r.ok, r.errors


def test_same_zone_twice_rejected():
    r = check_loc({"plan": [{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
                            {"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": "zone_a"}]},
                  INITIAL)
    assert not r.ok
    assert "red_cube" in r.errors[0]


def test_replace_object_in_its_own_zone_ok():
    r = check_loc({"plan": [{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"}]},
                  {**INITIAL, "red_cube": "zone_a"})
    assert r.ok, r.errors


def test_low_level_open_gripper_into_occupied_zone_rejected():
    r = check_loc({"plan": [{"skill": "move_above", "object": "red_cube"},
                            {"skill": "close_gripper"},
                            {"skill": "move_to_zone", "zone": "zone_b"},
                            {"skill": "open_gripper"}]},
                  {**INITIAL, "yellow_cube": "zone_b"})
    assert not r.ok


def test_open_gripper_while_holding_needs_move_to_zone():
    r = check_loc({"plan": [{"skill": "open_gripper"}]}, INITIAL, held="red_cube")
    assert not r.ok


# ---------------------------------------------------------------- temporary_position, 5 khoi
TEMP = "temporary_position"
# De bai: zone_b dang co blue_cube, nguoi dung "Put the red cube in Zone B."
DEMO = {**INITIAL, "blue_cube": "zone_b", "green_cube": "zone_c"}
DEMO_PLAN = {"plan": [{"skill": "check_zone", "zone": "zone_b"},
                      {"skill": "pick", "object": "blue_cube"},
                      {"skill": "find_free_position"},
                      {"skill": "place", "object": "blue_cube", "zone": TEMP},
                      {"skill": "pick", "object": "red_cube"},
                      {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                      {"skill": "home"}]}


def test_demo_plan_from_assignment_ok():
    r = check_loc(DEMO_PLAN, DEMO)
    assert r.ok, r.errors
    assert r.steps[0].expect == "blue_cube"   # check_zone du kien thay blue_cube


def test_check_zone_expectation_follows_plan():
    r = check_loc({"plan": [{"skill": "check_zone", "zone": "zone_a"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
                            {"skill": "check_zone", "zone": "zone_a"}]}, DEMO)
    assert r.ok, r.errors
    assert r.steps[0].expect == ""            # trong
    assert r.steps[3].expect == "red_cube"    # sau khi dat


def test_check_zone_without_scene_has_no_expectation():
    r = check({"plan": [{"skill": "check_zone", "zone": "zone_b"}]})
    assert r.ok and r.steps[0].expect is None


def test_temporary_position_needs_find_free_position():
    r = check_loc({"plan": [{"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": TEMP}]}, DEMO)
    assert not r.ok
    assert "find_free_position" in r.errors[0]


def test_temporary_position_used_once_per_find():
    r = check_loc({"plan": [{"skill": "find_free_position"},
                            {"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": TEMP},
                            {"skill": "pick", "object": "green_cube"},
                            {"skill": "place", "object": "green_cube", "zone": TEMP}]}, DEMO)
    assert not r.ok
    assert "Buoc 5" in r.errors[0]


def test_low_level_move_to_temporary_position_ok():
    r = check_loc({"plan": [{"skill": "move_above", "object": "blue_cube"},
                            {"skill": "close_gripper"},
                            {"skill": "find_free_position"},
                            {"skill": "move_to_zone", "zone": TEMP},
                            {"skill": "open_gripper"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_b"}]}, DEMO)
    assert r.ok, r.errors


def test_observation_steps_do_not_break_close_gripper():
    r = check({"plan": [{"skill": "move_above", "object": "red_cube"},
                        {"skill": "find_object", "object": "red_cube"},
                        {"skill": "detect_objects"},
                        {"skill": "close_gripper"}]})
    assert r.ok, r.errors


def test_temporary_position_not_a_zone_for_check_zone():
    r = check({"plan": [{"skill": "check_zone", "zone": TEMP}]})
    assert not r.ok


def test_observation_skills_reject_coordinates():
    assert not check({"plan": [{"skill": "find_free_position", "position": [0.2, 0.0]}]}).ok
    assert not check({"plan": [{"skill": "check_zone", "zone": "zone_a",
                                "object": "red_cube"}]}).ok


def test_place_on_table_removed():
    assert not check({"plan": [{"skill": "place_on_table", "object": "red_cube"}]}).ok


def test_temp_zone_no_longer_exists():
    r = check({"plan": [{"skill": "pick", "object": "red_cube"},
                        {"skill": "place", "object": "red_cube", "zone": "zone_tmp"}]})
    assert not r.ok
    assert "zone_tmp" in r.errors[0]


def test_personal_task_from_initial_world():
    # MSSV 23020764 (P = 4): A <- blue, B <- red, C <- yellow.
    # Ban dau blue o zone_b, green o zone_c -> blue chuyen thang sang zone_a, green ra o trong.
    r = check_loc({"plan": [{"skill": "check_zone", "zone": "zone_a"},
                            {"skill": "pick", "object": "blue_cube"},
                            {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                            {"skill": "check_zone", "zone": "zone_b"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                            {"skill": "check_zone", "zone": "zone_c"},
                            {"skill": "pick", "object": "green_cube"},
                            {"skill": "find_free_position"},
                            {"skill": "place", "object": "green_cube", "zone": TEMP},
                            {"skill": "pick", "object": "yellow_cube"},
                            {"skill": "place", "object": "yellow_cube", "zone": "zone_c"},
                            {"skill": "home"}]}, DEMO)
    assert r.ok, r.errors


def test_personal_task_wrong_order_rejected():
    # Dat yellow vao zone_c khi green van con o do
    r = check_loc({"plan": [{"skill": "pick", "object": "yellow_cube"},
                            {"skill": "place", "object": "yellow_cube", "zone": "zone_c"}]},
                  DEMO)
    assert not r.ok
    assert "green_cube" in r.errors[0]


def test_long_plan_for_five_blocks_allowed():
    steps = []
    for obj in OBJECTS:
        steps += [{"skill": "detect_objects"},
                  {"skill": "pick", "object": obj},
                  {"skill": "find_free_position"},
                  {"skill": "place", "object": obj, "zone": TEMP}]
    steps.append({"skill": "home"})  # 21 buoc
    assert check({"plan": steps}).ok


def test_plan_over_max_steps_rejected():
    assert not check({"plan": [{"skill": "home"}] * 41}).ok
