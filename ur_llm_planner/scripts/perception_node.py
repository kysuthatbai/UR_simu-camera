#!/usr/bin/env python3
"""Perception node: camera tren cao -> vi tri cac khoi + trang thai cac vung.

    /camera/image + /camera/camera_info + TF(world -> camera_optical_frame)
        -> phan doan mau HSV -> tam mat tren khoi -> giao tia voi mat ban -> (x, y) world
        -> khoi nao o vung nao, vung nao trong
        -> /perception/scene (ur_llm_planner/msg/SceneObservation)

Vi tri vat KHONG lay tu file cau hinh: chi co ten vat + mau sac (de biet dai HSV) va vi tri
cac vung (ha tang co dinh). Khoi bi che (vd: canh tay robot o phia tren) duoc giu vi tri quan
sat gan nhat voi visible = false. Khoi robot dang cam (/skill_server/held_object) khong duoc
tinh vao trang thai vung.
"""

import sys
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from llm_planner.perception import (CameraModel, HsvRange, detect_blob, expected_cube_area,
                                    quaternion_to_matrix, zone_occupancy, zone_of)
from ur_llm_planner.msg import DetectedObject, SceneObservation, ZoneState

# Dai HSV mac dinh (OpenCV: H 0-179), do tren anh render cua Gazebo (ogre2).
DEFAULT_HSV = {
    "red": [170, 8, 120, 50],
    "yellow": [20, 36, 120, 50],
    "green": [50, 80, 120, 50],
    "blue": [105, 125, 130, 50],
    "purple": [135, 160, 120, 50],
}


class ObjectTrack:
    def __init__(self):
        self.position: Optional[np.ndarray] = None
        self.visible = False
        self.last_seen = Time()
        self.area = 0.0
        self.blob = None


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.cube_size = self.declare_parameter("cube_size", 0.04).value
        zone_size = self.declare_parameter("zone_size", 0.08).value
        table_size = self.declare_parameter("table_size", [0.90, 1.00, 0.75]).value
        table_center = self.declare_parameter("table_center", [0.15, 0.0, -0.375]).value

        names = [n for n in self.declare_parameter("object_names", [""]).value if n]
        zone_names = [n for n in self.declare_parameter("zone_names", [""]).value if n]
        if not names or not zone_names:
            raise RuntimeError("Chua khai bao object_names / zone_names (nap config/scene.yaml)")

        p = "perception."
        self.camera_frame = self.declare_parameter(p + "camera_frame",
                                                   "camera_optical_frame").value
        image_topic = self.declare_parameter(p + "image_topic", "/camera/image").value
        info_topic = self.declare_parameter(p + "camera_info_topic", "/camera/camera_info").value
        self.min_area_ratio = self.declare_parameter(p + "min_area_ratio", 0.6).value
        self.max_area_ratio = self.declare_parameter(p + "max_area_ratio", 2.5).value
        self.top_face_ratio = self.declare_parameter(p + "top_face_ratio", 0.85).value
        zone_margin = self.declare_parameter(p + "zone_margin", 0.01).value

        self.colors: Dict[str, HsvRange] = {}
        for name in names:
            color = self.declare_parameter(f"objects.{name}.color", name.split("_")[0]).value
            key = f"{p}hsv.{color}"
            values = self.declare_parameter(key, DEFAULT_HSV.get(color, [0, 0, 0, 0])).value
            if list(values) == [0, 0, 0, 0]:
                raise RuntimeError(f"Chua khai bao dai HSV '{key}' cho {name}")
            self.colors[name] = HsvRange.from_list(values)

        self.zones = {z: self.declare_parameter(f"zones.{z}.position", [0.0, 0.0, 0.0]).value
                      for z in zone_names}
        self.zone_half = zone_size / 2.0 + zone_margin
        half = [table_size[0] / 2.0, table_size[1] / 2.0]
        self.table_bounds = (table_center[0] - half[0], table_center[0] + half[0],
                             table_center[1] - half[1], table_center[1] + half[1])

        self.tracks = {n: ObjectTrack() for n in names}
        self.held = ""
        self.info: Optional[CameraInfo] = None
        self.last_summary = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, info_topic, self._on_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, 2)
        self.create_subscription(String, "/skill_server/held_object", self._on_held, latched)
        self.scene_pub = self.create_publisher(SceneObservation, "perception/scene", latched)
        self.debug_pub = self.create_publisher(Image, "perception/debug_image", 2)
        self.get_logger().info(
            f"Perception: {image_topic} | {len(names)} khoi: {', '.join(names)} | "
            f"vung: {', '.join(zone_names)}")

    # ------------------------------------------------------------------ callbacks
    def _on_info(self, msg: CameraInfo):
        self.info = msg

    def _on_held(self, msg: String):
        # Vat vua duoc gap: vi tri cu khong con dung. Sau khi tha, vat nam duoi gripper (bi che)
        # -> bao "chua thay" cho toi khi camera nhin thay vat o cho moi.
        if msg.data in self.tracks:
            self.tracks[msg.data].position = None
        self.held = msg.data

    def _camera(self) -> Optional[CameraModel]:
        if self.info is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.camera_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"Chua co TF {self.world_frame} -> {self.camera_frame}: {e}",
                                   throttle_duration_sec=5.0)
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        k = self.info.k
        return CameraModel(fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                           rotation=quaternion_to_matrix(q.x, q.y, q.z, q.w),
                           translation=np.array([t.x, t.y, t.z]))

    def _on_image(self, msg: Image):
        camera = self._camera()
        if camera is None:
            return
        rgb = image_to_array(msg)
        if rgb is None:
            self.get_logger().error(f"Khong ho tro encoding '{msg.encoding}'",
                                    throttle_duration_sec=5.0)
            return
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        expected = expected_cube_area(camera, self.cube_size)
        stamp = Time.from_msg(msg.header.stamp)
        xmin, xmax, ymin, ymax = self.table_bounds

        for name, track in self.tracks.items():
            blob = detect_blob(hsv, self.colors[name], self.min_area_ratio * expected,
                               self.max_area_ratio * expected, self.top_face_ratio)
            track.visible, track.blob = False, blob
            if blob is None or name == self.held:
                continue
            point = camera.pixel_to_plane(blob.u, blob.v, self.cube_size)
            if point is None or not (xmin <= point[0] <= xmax and ymin <= point[1] <= ymax):
                continue
            track.position = np.array([point[0], point[1], self.cube_size / 2.0])
            track.visible, track.area, track.last_seen = True, blob.area, stamp

        self._publish(msg.header.stamp)
        if self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(msg, rgb, camera)

    # ------------------------------------------------------------------ output
    def _locations(self) -> Dict[str, Optional[str]]:
        return {n: (zone_of(t.position, self.zones, self.zone_half)
                    if t.position is not None and n != self.held else None)
                for n, t in self.tracks.items()}

    def _publish(self, stamp):
        locations = self._locations()
        occupancy = zone_occupancy(locations, self.zones)
        out = SceneObservation()
        out.header.stamp = stamp
        out.header.frame_id = self.world_frame
        for name, track in self.tracks.items():
            obj = DetectedObject(name=name, visible=track.visible,
                                 known=track.position is not None,
                                 in_zone=locations[name] or "", pixel_area=float(track.area))
            if track.position is not None:
                obj.position = Point(x=float(track.position[0]), y=float(track.position[1]),
                                     z=float(track.position[2]))
                obj.last_seen = track.last_seen.to_msg()
            out.objects.append(obj)
        for zone, objects in occupancy.items():
            out.zones.append(ZoneState(name=zone, occupied=bool(objects),
                                       object=objects[0] if objects else ""))
        self.scene_pub.publish(out)

        summary = " | ".join(f"{z}: {', '.join(o) if o else 'trong'}"
                             for z, o in occupancy.items())
        missing = [n for n, t in self.tracks.items() if t.position is None and n != self.held]
        if missing:
            summary += f" | chua thay: {', '.join(missing)}"
        if summary != self.last_summary:
            self.last_summary = summary
            self.get_logger().info(f"[Camera] {summary}")
        for zone, objects in occupancy.items():
            if len(objects) > 1:
                self.get_logger().warn(f"{zone} co nhieu vat: {objects}",
                                       throttle_duration_sec=5.0)

    def _publish_debug(self, msg: Image, rgb: np.ndarray, camera: CameraModel):
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        half = self.zone_half
        for zone, c in self.zones.items():
            corners = [camera.world_to_pixel((c[0] + dx, c[1] + dy, 0.0))
                       for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half))]
            if all(corners):
                pts = np.array(corners, dtype=np.int32)
                cv2.polylines(img, [pts], True, (0, 200, 255), 1)
                cv2.putText(img, zone, tuple(pts[0] + [0, -4]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (0, 200, 255), 1)
        for name, track in self.tracks.items():
            if track.blob is None:
                continue
            color = (0, 255, 0) if track.visible else (0, 0, 255)
            cv2.drawContours(img, [track.blob.contour], -1, color, 1)
            cv2.circle(img, (int(round(track.blob.u)), int(round(track.blob.v))), 2, color, -1)
            label = name if track.position is None else \
                f"{name} ({track.position[0]:.3f}, {track.position[1]:.3f})"
            cv2.putText(img, label, (int(track.blob.u) + 8, int(track.blob.v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        out = Image(header=msg.header, height=img.shape[0], width=img.shape[1],
                    encoding="bgr8", is_bigendian=0, step=img.shape[1] * 3)
        out.data = img.tobytes()
        self.debug_pub.publish(out)


def image_to_array(msg: Image) -> Optional[np.ndarray]:
    """sensor_msgs/Image (rgb8/bgr8) -> mang RGB (H, W, 3)."""
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = data[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img if msg.encoding == "rgb8" else img[:, :, ::-1]


def main():
    rclpy.init(args=sys.argv)
    try:
        node = PerceptionNode()
    except (RuntimeError, ValueError) as e:
        print(e)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
