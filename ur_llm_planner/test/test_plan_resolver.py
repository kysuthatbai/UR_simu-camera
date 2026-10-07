import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.plan_resolver import resolve_plan  # noqa: E402
from llm_planner.plan_validator import validate_plan  # noqa: E402

OBJECTS = ["red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"]
ZONES = ["zone_a", "zone_b", "zone_c"]
INITIAL = {o: None for o in OBJECTS}
# De bai: zone_b dang co blue_cube
DEMO = {**INITIAL, "blue_cube": "zone_b", "green_cube": "zone_c"}


def steps_of(plan, held=None):
    r = validate_plan({"plan": plan}, OBJECTS, ZONES, held)   # cu phap, khong xet vung
    assert r.ok, r.errors
    return r.steps


def resolve(plan, locations, held=None):
    steps, notes = resolve_plan(steps_of(plan), ZONES, held, locations)
    return [str(s) for s in steps], steps, notes


def test_naive_plan_gets_zone_cleared_like_assignment_example():
    # LLM chi sinh pick -> place -> home; he thong phai tu don zone_b
    labels, steps, notes = resolve([{"skill": "pick", "object": "red_cube"},
                                    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                                    {"skill": "home"}], DEMO)
    assert labels == ["check_zone(zone_b)", "pick(blue_cube)", "find_free_position()",
                      "place(blue_cube, temporary_position)", "pick(red_cube)",
                      "place(red_cube, zone_b)", "home()"]
    assert [s.inserted for s in steps] == [True, True, True, True, False, False, False]
    assert any("blue_cube" in n for n in notes)
    # Ke hoach sau khi bo sung phai hop le voi trang thai camera
    final = validate_plan({"plan": [s.to_dict() for s in steps]}, OBJECTS, ZONES, None, DEMO)
    assert final.ok, final.errors
    assert final.steps[0].expect == "blue_cube"


def test_free_zone_only_gets_check_zone():
    labels, _, _ = resolve([{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"}], DEMO)
    assert labels == ["check_zone(zone_a)", "pick(red_cube)", "place(red_cube, zone_a)"]


def test_complete_llm_plan_left_unchanged():
    plan = [{"skill": "check_zone", "zone": "zone_b"},
            {"skill": "pick", "object": "blue_cube"},
            {"skill": "find_free_position"},
            {"skill": "place", "object": "blue_cube", "zone": "temporary_position"},
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"}]
    labels, steps, notes = resolve(plan, DEMO)
    assert not notes and not any(s.inserted for s in steps)
    assert len(labels) == len(plan)


def test_llm_clearing_to_other_zone_respected():
    # LLM tu chuyen blue sang zone_a (trong) -> khong chen don vung nua
    labels, _, notes = resolve([{"skill": "check_zone", "zone": "zone_b"},
                                {"skill": "pick", "object": "blue_cube"},
                                {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                                {"skill": "pick", "object": "red_cube"},
                                {"skill": "place", "object": "red_cube", "zone": "zone_b"}],
                               DEMO)
    assert "find_free_position()" not in labels
    assert labels.count("check_zone(zone_b)") == 1
    assert labels.index("check_zone(zone_a)") < labels.index("pick(blue_cube)")


def test_zone_rechecked_after_something_placed_into_it():
    labels, _, _ = resolve([{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"},
                            {"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_a"}], DEMO)
    assert labels.count("check_zone(zone_a)") == 2


def test_two_occupied_zones_each_cleared():
    labels, steps, _ = resolve([{"skill": "pick", "object": "red_cube"},
                                {"skill": "place", "object": "red_cube", "zone": "zone_c"},
                                {"skill": "pick", "object": "purple_cube"},
                                {"skill": "place", "object": "purple_cube", "zone": "zone_b"}],
                               DEMO)
    assert labels.count("find_free_position()") == 2
    final = validate_plan({"plan": [s.to_dict() for s in steps]}, OBJECTS, ZONES, None, DEMO)
    assert final.ok, final.errors


def test_object_already_in_target_zone_not_moved_away():
    labels, _, notes = resolve([{"skill": "pick", "object": "blue_cube"},
                                {"skill": "place", "object": "blue_cube", "zone": "zone_b"}],
                               DEMO)
    assert labels == ["check_zone(zone_b)", "pick(blue_cube)", "place(blue_cube, zone_b)"]


def test_without_scene_only_checks_zone():
    labels, _, _ = resolve([{"skill": "pick", "object": "red_cube"},
                            {"skill": "place", "object": "red_cube", "zone": "zone_b"}], None)
    assert labels == ["check_zone(zone_b)", "pick(red_cube)", "place(red_cube, zone_b)"]


def test_holding_object_at_start_cannot_clear_and_validator_rejects():
    steps, _ = resolve_plan(steps_of([{"skill": "place", "object": "red_cube",
                                       "zone": "zone_b"}], held="red_cube"),
                            ZONES, "red_cube", DEMO)
    assert str(steps[0]) == "check_zone(zone_b)"
    final = validate_plan({"plan": [s.to_dict() for s in steps]}, OBJECTS, ZONES,
                          "red_cube", DEMO)
    assert not final.ok
