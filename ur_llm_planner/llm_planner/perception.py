"""Thuat toan nhan dien khoi tu anh camera (khong phu thuoc ROS -> kiem thu duoc bang pytest).

Camera RGB co dinh tren cao, nhin thang xuong mat ban. Voi moi khoi:
  1. Phan doan theo mau trong khong gian HSV (moi khoi mot dai mau).
  2. Lay thanh phan lien thong lon nhat, loc theo dien tich ky vong.
  3. Chi giu vung sang nhat cua blob (mat tren cua khoi, duoc chieu sang truc tiep). Camera
     nhin thay them mot mat ben toi cua khoi -> neu lay ca blob thi tam bi lech vai mm.
  4. Tam mat tren (u, v) -> tia tu tam camera -> giao voi mat phang z = cube_size (mat tren
     khoi) -> toa do (x, y) trong frame world. Tam khoi o z = cube_size / 2.
  5. Gan khoi vao vung neu tam khoi nam trong o vuong cua vung.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvRange:
    """Dai mau HSV theo thang OpenCV (H: 0-179, S/V: 0-255).

    h_low > h_high nghia la dai mau vong qua 0 (vd: do = [170..179] + [0..8]).
    """
    h_low: int
    h_high: int
    s_min: int
    v_min: int

    @staticmethod
    def from_list(values: Sequence[float]) -> "HsvRange":
        if len(values) != 4:
            raise ValueError(f"Dai HSV phai co 4 gia tri [h_low, h_high, s_min, v_min]: {values}")
        return HsvRange(*(int(v) for v in values))

    def mask(self, hsv: np.ndarray) -> np.ndarray:
        if self.h_low <= self.h_high:
            return cv2.inRange(hsv, (self.h_low, self.s_min, self.v_min),
                               (self.h_high, 255, 255))
        return cv2.bitwise_or(
            cv2.inRange(hsv, (self.h_low, self.s_min, self.v_min), (179, 255, 255)),
            cv2.inRange(hsv, (0, self.s_min, self.v_min), (self.h_high, 255, 255)))


@dataclass
class Blob:
    u: float          # tam mat tren (pixel, cot)
    v: float          # tam mat tren (pixel, hang)
    area: float       # dien tich ca blob (pixel)
    contour: np.ndarray


_KERNEL = np.ones((3, 3), np.uint8)


def detect_blob(hsv: np.ndarray, color: HsvRange, min_area: float, max_area: float,
                top_face_ratio: float = 0.85) -> Optional[Blob]:
    """Tim khoi co mau `color` trong anh HSV. Tra ve None neu khong thay (hoac bi che mot phan)."""
    mask = cv2.morphologyEx(color.mask(hsv), cv2.MORPH_OPEN, _KERNEL)
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = float(stats[best, cv2.CC_STAT_AREA])
    if not (min_area <= area <= max_area):
        return None
    blob_mask = (labels == best)

    # Mat tren: cac pixel sang nhat cua blob (mat ben khong duoc chieu sang truc tiep)
    values = hsv[:, :, 2][blob_mask]
    threshold = top_face_ratio * float(np.percentile(values, 90))
    top = (blob_mask & (hsv[:, :, 2] >= threshold)).astype(np.uint8)
    n_top, top_labels, top_stats, top_centroids = cv2.connectedComponentsWithStats(top)
    if n_top > 1 and top_stats[1:, cv2.CC_STAT_AREA].max() >= 0.3 * area:
        k = 1 + int(np.argmax(top_stats[1:, cv2.CC_STAT_AREA]))
        u, v = top_centroids[k]
    else:  # khong tach duoc mat tren -> dung tam ca blob
        ys, xs = np.nonzero(blob_mask)
        u, v = float(xs.mean()), float(ys.mean())

    contours, _ = cv2.findContours(blob_mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    return Blob(float(u), float(v), area, contours[0] if contours else np.empty((0, 1, 2)))


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


@dataclass
class CameraModel:
    """Camera pinhole + pose cua optical frame trong world (z nhin ra truoc, x phai, y xuong)."""
    fx: float
    fy: float
    cx: float
    cy: float
    rotation: np.ndarray      # R: world <- optical
    translation: np.ndarray   # vi tri tam camera trong world

    def pixel_to_plane(self, u: float, v: float, plane_z: float) -> Optional[np.ndarray]:
        """Giao tia qua pixel (u, v) voi mat phang z = plane_z cua world."""
        ray = self.rotation @ np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        if abs(ray[2]) < 1e-9:
            return None
        s = (plane_z - self.translation[2]) / ray[2]
        if s <= 0:
            return None
        return self.translation + s * ray

    def world_to_pixel(self, point: Sequence[float]) -> Optional[Tuple[float, float]]:
        p = self.rotation.T @ (np.asarray(point, dtype=float) - self.translation)
        if p[2] <= 1e-9:
            return None
        return self.fx * p[0] / p[2] + self.cx, self.fy * p[1] / p[2] + self.cy


def expected_cube_area(camera: CameraModel, cube_size: float) -> float:
    """Dien tich (pixel) mat tren cua khoi nam tren ban, nhin tu camera."""
    depth = camera.translation[2] - cube_size
    return (cube_size * camera.fx / depth) * (cube_size * camera.fy / depth)


def zone_of(xy: Sequence[float], zones: Mapping[str, Sequence[float]], half_size: float
            ) -> Optional[str]:
    """Vung chua diem (x, y); None neu nam ngoai moi vung."""
    for name, center in zones.items():
        if abs(xy[0] - center[0]) <= half_size and abs(xy[1] - center[1]) <= half_size:
            return name
    return None


def zone_occupancy(locations: Mapping[str, Optional[str]], zone_names: Iterable[str]
                   ) -> Dict[str, List[str]]:
    """Vung -> danh sach vat dang nam trong vung (rong = vung trong)."""
    result: Dict[str, List[str]] = {z: [] for z in zone_names}
    for obj, zone in locations.items():
        if zone in result:
            result[zone].append(obj)
    return result
