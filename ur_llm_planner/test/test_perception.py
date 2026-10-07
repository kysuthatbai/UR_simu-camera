import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.perception import (CameraModel, HsvRange, detect_blob,  # noqa: E402
                                    expected_cube_area, quaternion_to_matrix, zone_occupancy,
                                    zone_of)

# Camera tren cao nhu trong worlds/pick_place.sdf: (0.15, 0, 1.5), nhin thang xuong.
# Optical frame trong world: x = +Y, y = +X, z = -Z.
NADIR = np.array([[0.0, 1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0]])
FX = 876.6439


def nadir_camera():
    return CameraModel(fx=FX, fy=FX, cx=320.0, cy=240.0, rotation=NADIR,
                       translation=np.array([0.15, 0.0, 1.5]))


ZONES = {"zone_a": [0.282, 0.103, 0.0], "zone_b": [0.212, 0.212, 0.0],
         "zone_c": [0.103, 0.282, 0.0]}


# ---------------------------------------------------------------- hinh hoc camera
def test_quaternion_identity_and_rotation():
    assert np.allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3))
    s = np.sqrt(0.5)  # quay 90 do quanh z: x -> y
    assert np.allclose(quaternion_to_matrix(0, 0, s, s) @ [1, 0, 0], [0, 1, 0])


def test_image_center_maps_below_camera():
    p = nadir_camera().pixel_to_plane(320.0, 240.0, 0.04)
    assert np.allclose(p, [0.15, 0.0, 0.04])


def test_image_axes_match_world_axes():
    cam = nadir_camera()
    right = cam.pixel_to_plane(420.0, 240.0, 0.04)   # u tang -> +Y world
    down = cam.pixel_to_plane(320.0, 340.0, 0.04)    # v tang -> +X world
    assert right[1] > 0 and np.isclose(right[0], 0.15)
    assert down[0] > 0.15 and np.isclose(down[1], 0.0)


@pytest.mark.parametrize("point", [(0.212, 0.212, 0.04), (0.150, -0.260, 0.04),
                                   (0.0, 0.3, 0.04), (0.33, -0.1, 0.04)])
def test_pixel_world_round_trip(point):
    cam = nadir_camera()
    u, v = cam.world_to_pixel(point)
    assert np.allclose(cam.pixel_to_plane(u, v, point[2]), point, atol=1e-9)


def test_ray_parallel_or_behind_plane_rejected():
    cam = nadir_camera()
    assert cam.pixel_to_plane(320.0, 240.0, 2.0) is None  # mat phang o tren camera


def test_expected_cube_area():
    side = 0.04 * FX / (1.5 - 0.04)  # ~24 pixel
    assert np.isclose(expected_cube_area(nadir_camera(), 0.04), side * side)


# ---------------------------------------------------------------- phan doan mau
def render(squares, size=(480, 640)):
    """Anh BGR nen xam; squares = [(u0, v0, w, h, bgr)]."""
    img = np.full((*size, 3), 80, np.uint8)
    for u0, v0, w, h, bgr in squares:
        img[v0:v0 + h, u0:u0 + w] = bgr
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


RED = HsvRange(170, 8, 120, 50)
BLUE = HsvRange(105, 125, 130, 50)


def test_red_range_wraps_around_zero():
    hsv = render([(100, 100, 24, 24, (20, 20, 230)),     # H ~ 0
                  (300, 100, 24, 24, (60, 20, 230))])    # H ~ 175
    assert cv2.countNonZero(RED.mask(hsv)) == 2 * 24 * 24


def test_detect_blob_uses_bright_top_face():
    # Mat tren sang (24x24) + mat ben toi (10x24) ben trai -> tam phai nam giua mat tren
    hsv = render([(290, 200, 10, 24, (20, 20, 120)), (300, 200, 24, 24, (20, 20, 230))])
    blob = detect_blob(hsv, RED, min_area=300, max_area=2000)
    assert blob is not None
    assert blob.area == 34 * 24
    assert abs(blob.u - 311.5) < 0.5 and abs(blob.v - 211.5) < 0.5


def test_detect_blob_rejects_small_partially_hidden_blob():
    hsv = render([(300, 200, 8, 8, (20, 20, 230))])
    assert detect_blob(hsv, RED, min_area=300, max_area=2000) is None


def test_detect_blob_ignores_other_colors_and_gray():
    hsv = render([(300, 200, 24, 24, (230, 60, 20)),     # xanh duong
                  (100, 100, 24, 24, (200, 200, 200))])  # tam vung xam sang
    assert detect_blob(hsv, RED, 300, 2000) is None
    blob = detect_blob(hsv, BLUE, 300, 2000)
    assert blob is not None and abs(blob.u - 311.5) < 0.5


def test_low_saturation_robot_blue_not_detected():
    # Mau nap khop UR3e (0.49, 0.68, 0.80) co do bao hoa ~0.39 -> khong phai blue_cube
    hsv = render([(300, 200, 40, 40, (204, 173, 125))])
    assert detect_blob(hsv, BLUE, 300, 3000) is None


def test_invalid_hsv_list():
    with pytest.raises(ValueError):
        HsvRange.from_list([1, 2, 3])


# ---------------------------------------------------------------- vung
def test_zone_of():
    half = 0.05
    assert zone_of((0.212, 0.212), ZONES, half) == "zone_b"
    assert zone_of((0.215, 0.205), ZONES, half) == "zone_b"
    assert zone_of((0.150, -0.260), ZONES, half) is None


def test_zone_occupancy_empty_and_occupied():
    occ = zone_occupancy({"red_cube": "zone_b", "green_cube": "zone_c", "blue_cube": None},
                         ZONES)
    assert occ == {"zone_a": [], "zone_b": ["red_cube"], "zone_c": ["green_cube"]}
