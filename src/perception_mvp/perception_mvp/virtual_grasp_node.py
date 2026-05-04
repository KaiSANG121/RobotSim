import math
import shutil
import subprocess
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class VirtualGraspNode(Node):
    """
    Lightweight Gazebo-only grasp helper.

    Gazebo Fortress does not expose a ready-made fixed-joint attach service in
    this world, so for the MVP we keep the carried object's model pose aligned
    with the gripper frame after the simulated close succeeds, then release it
    when the state machine opens the gripper.
    """

    TARGETS: Dict[str, Dict[str, float | str]] = {
        "red_cylinder": {
            "model_name": "test_can",
            "origin_z_below_detection": 0.055,
            "default_z_offset": -0.090,
            "yaw": 0.0,
        },
        "yellow_box": {
            "model_name": "test_box",
            "origin_z_below_detection": 0.035,
            "default_z_offset": -0.075,
            "yaw": 0.4,
        },
        "brown_cube": {
            "model_name": "test_brown_cube",
            "origin_z_below_detection": 0.0325,
            "default_z_offset": -0.073,
            "yaw": 0.0,
        },
    }

    ATTACH_STATES = {
        "LIFT_AFTER_GRASP",
        "MOVE_TO_PLACE_HOVER",
        "MOVE_DECLUTTER_TEMP_PLACE",
    }
    DETACH_STATES = {"NEXT_TARGET", "RESTORE_ORIGINAL_TARGET", "FINISHED"}

    def __init__(self):
        super().__init__("virtual_grasp_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("gripper_frame", "gripper_center")
        self.declare_parameter("gazebo_set_pose_service", "/world/default/set_pose")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("service_timeout_ms", 1000)
        self.declare_parameter("warn_period_sec", 3.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.gripper_frame = str(self.get_parameter("gripper_frame").value)
        self.set_pose_service = str(
            self.get_parameter("gazebo_set_pose_service").value
        )
        update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.service_timeout_ms = int(self.get_parameter("service_timeout_ms").value)
        self.warn_period_sec = float(self.get_parameter("warn_period_sec").value)

        self.ign_cmd = shutil.which("ign")
        if self.ign_cmd is None:
            self.get_logger().error(
                "ign CLI not found; virtual grasp is disabled."
            )
            self.enabled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_target_name: str = ""
        self.latest_target_point: Optional[PointStamped] = None
        self.state: str = ""

        self.attached = False
        self.attached_target_name: str = ""
        self.attached_model_name: str = ""
        self.attach_offset = [0.0, 0.0, -0.08]
        self.attached_yaw = 0.0
        self._last_warn_time = {}

        self.create_subscription(
            String,
            "/task/current_target_name",
            self.current_target_name_callback,
            10,
        )
        self.create_subscription(
            String,
            "/task/state",
            self.state_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/perception/current_target_point_world",
            self.target_point_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / update_rate_hz, self.update_attached_pose)

        self.get_logger().info(
            "virtual_grasp_node started. "
            f"enabled={self.enabled}, service={self.set_pose_service}, "
            f"rate={update_rate_hz:.1f}Hz"
        )

    def current_target_name_callback(self, msg: String):
        target_name = msg.data.strip()
        if target_name != self.current_target_name:
            self.latest_target_point = None
        self.current_target_name = target_name

    def target_point_callback(self, msg: PointStamped):
        self.latest_target_point = msg

    def state_callback(self, msg: String):
        new_state = msg.data.strip()
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        if not self.enabled:
            return

        if new_state in self.ATTACH_STATES and not self.attached:
            self.attach_current_target()
        elif new_state in self.DETACH_STATES and self.attached:
            self.detach_current_target(f"state {old_state} -> {new_state}")

    def attach_current_target(self):
        target_name = self.current_target_name
        if target_name not in self.TARGETS:
            self.warn_throttled(
                "unknown_target",
                f"Cannot attach unknown target '{target_name}'.",
            )
            return

        transform = self.lookup_gripper_transform()
        if transform is None:
            return

        cfg = self.TARGETS[target_name]
        model_name = str(cfg["model_name"])
        gripper_xyz = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]

        if self.latest_target_point is not None:
            p = self.latest_target_point.point
            model_origin = [
                float(p.x),
                float(p.y),
                float(p.z) - float(cfg["origin_z_below_detection"]),
            ]
            self.attach_offset = [
                model_origin[0] - gripper_xyz[0],
                model_origin[1] - gripper_xyz[1],
                model_origin[2] - gripper_xyz[2],
            ]
        else:
            self.attach_offset = [0.0, 0.0, float(cfg["default_z_offset"])]

        self.attached = True
        self.attached_target_name = target_name
        self.attached_model_name = model_name
        self.attached_yaw = float(cfg["yaw"])

        self.get_logger().info(
            f"Attached target={target_name}, model={model_name}, "
            f"offset={[round(v, 3) for v in self.attach_offset]}"
        )
        self.update_attached_pose()

    def detach_current_target(self, reason: str):
        self.update_attached_pose()
        self.get_logger().info(
            f"Detached target={self.attached_target_name}, "
            f"model={self.attached_model_name}, reason={reason}"
        )
        self.attached = False
        self.attached_target_name = ""
        self.attached_model_name = ""

    def update_attached_pose(self):
        if not self.enabled or not self.attached:
            return

        transform = self.lookup_gripper_transform()
        if transform is None:
            return

        t = transform.transform.translation
        position = [
            float(t.x) + self.attach_offset[0],
            float(t.y) + self.attach_offset[1],
            float(t.z) + self.attach_offset[2],
        ]

        ok = self.set_model_pose(
            model_name=self.attached_model_name,
            position=position,
            yaw=self.attached_yaw,
        )
        if not ok:
            self.warn_throttled(
                "set_pose_failed",
                f"Failed to update pose for model={self.attached_model_name}.",
            )

    def lookup_gripper_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.world_frame,
                self.gripper_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.warn_throttled(
                "tf_missing",
                f"Cannot lookup {self.world_frame}->{self.gripper_frame}: {exc}",
            )
            return None

    def set_model_pose(self, model_name: str, position, yaw: float) -> bool:
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        request = (
            f'name: "{model_name}" '
            f'position {{ x: {position[0]:.6f} y: {position[1]:.6f} '
            f'z: {position[2]:.6f} }} '
            f'orientation {{ x: 0 y: 0 z: {qz:.6f} w: {qw:.6f} }}'
        )

        try:
            result = subprocess.run(
                [
                    self.ign_cmd,
                    "service",
                    "-s",
                    self.set_pose_service,
                    "--reqtype",
                    "ignition.msgs.Pose",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    str(self.service_timeout_ms),
                    "-r",
                    request,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(1.0, self.service_timeout_ms / 1000.0 + 0.5),
            )
        except Exception as exc:
            self.warn_throttled("ign_exception", f"ign service call failed: {exc}")
            return False

        if result.returncode != 0:
            self.warn_throttled(
                "ign_returncode",
                f"ign service returned {result.returncode}: {result.stderr.strip()}",
            )
            return False

        text = f"{result.stdout}\n{result.stderr}".lower()
        return "false" not in text

    def warn_throttled(self, key: str, text: str):
        now = time.monotonic()
        last = self._last_warn_time.get(key, 0.0)
        if now - last < self.warn_period_sec:
            return
        self._last_warn_time[key] = now
        self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
