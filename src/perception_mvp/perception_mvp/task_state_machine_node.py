import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Float64, Int32, String


def euler_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def rotate_vector_by_rpy_deg(
    vector: List[float],
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> List[float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    x, y, z = vector
    return [
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y
        + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y
        + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    ]


class TaskStateMachineNode(Node):
    def __init__(self):
        super().__init__("task_state_machine_node")

        # ==================================================
        # Parameters
        # ==================================================
        self.declare_parameter(
            "target_sequence",
            ["red_cylinder", "yellow_box", "brown_cube"],
        )
        self.declare_parameter("target_frame", "world")

        self.declare_parameter("loop_period_sec", 0.1)
        self.declare_parameter("target_settle_sec", 0.8)
        self.declare_parameter("target_visible_timeout_sec", 3.0)
        self.declare_parameter("declutter_candidate_visible_timeout_sec", 1.5)
        self.declare_parameter("executor_match_timeout_sec", 5.0)

        self.declare_parameter("verify_pregrasp_wait_sec", 0.5)
        self.declare_parameter("grasp_height_offset", 0.035)
        self.declare_parameter("grasp_height_offsets", [0.035])
        self.declare_parameter("grasp_position_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("grasp_reference_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("grasp_rpy_deg", [180.0, 90.0, 0.0])
        # Gripper action uses sim time, while this state machine uses wall time.
        # On slow Gazebo, 3s sim time can exceed 15s wall time, so this fallback
        # must stay longer than the executor's gripper_result_timeout_sec.
        self.declare_parameter("virtual_grasp_wait_sec", 35.0)
        self.declare_parameter("virtual_release_wait_sec", 35.0)
        self.declare_parameter("max_gripper_retries", 2)

        self.declare_parameter("max_grasp_retries", 1)
        self.declare_parameter("max_declutter_per_target", 1)
        self.declare_parameter("max_place_retries", 1)

        # Bin B hover pose. The y value stops a little before the bin center so
        # the carried object keeps more margin from the far wall during release.
        self.declare_parameter("place_xyz", [-0.36, -0.145, 0.28])
        self.declare_parameter("place_slots_xyz", [-0.36, -0.145, 0.28])
        # Keep place hover orientation aligned with grasp_rpy to avoid a large
        # wrist flip from pregrasp to Bin B.
        self.declare_parameter("place_rpy_deg", [180.0, 90.0, 0.0])

        # Declutter temporary hover pose
        self.declare_parameter("declutter_temp_xyz", [-0.36, -0.145, 0.28])
        # Same orientation as place/grasp; declutter uses the same hover target.
        self.declare_parameter("declutter_temp_rpy_deg", [180.0, 90.0, 0.0])

        # Lift after closing before moving laterally between bins. This keeps
        # the virtual attached object above the bin walls during transport.
        self.declare_parameter("carry_lift_enabled", True)
        self.declare_parameter("carry_lift_z", 0.32)

        # Gripper control positions: 0.0 = open, 0.05 = closed (FR-5.2)
        self.declare_parameter("gripper_close_position", 0.05)
        self.declare_parameter("gripper_open_position", 0.0)
        # Partial-open opening used during the descent before closing.
        # 0=fully open (~10.3cm gap, ~15.5cm outer pad span);
        # 0.010 → ~8.3cm gap (still wider than the largest object 7cm) but
        # ~13.5cm outer span, so the finger pads stick out 1cm less per side
        # while descending past neighbor objects.
        self.declare_parameter("gripper_approach_position", 0.010)
        # Wait budget for the partial-open command before falling back to MOVE_TO_GRASP.
        self.declare_parameter("gripper_approach_wait_sec", 20.0)

        self.target_sequence: List[str] = list(
            self.get_parameter("target_sequence").value
        )
        self.target_frame: str = str(self.get_parameter("target_frame").value)

        self.loop_period_sec: float = float(
            self.get_parameter("loop_period_sec").value
        )
        self.target_settle_sec: float = float(
            self.get_parameter("target_settle_sec").value
        )
        self.target_visible_timeout_sec: float = float(
            self.get_parameter("target_visible_timeout_sec").value
        )
        self.declutter_candidate_visible_timeout_sec: float = float(
            self.get_parameter("declutter_candidate_visible_timeout_sec").value
        )
        self.executor_match_timeout_sec: float = float(
            self.get_parameter("executor_match_timeout_sec").value
        )

        self.verify_pregrasp_wait_sec: float = float(
            self.get_parameter("verify_pregrasp_wait_sec").value
        )
        self.grasp_height_offset: float = float(
            self.get_parameter("grasp_height_offset").value
        )
        self.grasp_height_offsets = [
            float(v) for v in self.get_parameter("grasp_height_offsets").value
        ]
        self.grasp_position_offset_xyz = [
            float(v) for v in self.get_parameter("grasp_position_offset_xyz").value
        ]
        if len(self.grasp_position_offset_xyz) != 3:
            self.grasp_position_offset_xyz = [0.0, 0.0, 0.0]
        self.grasp_reference_offset_xyz = [
            float(v) for v in self.get_parameter("grasp_reference_offset_xyz").value
        ]
        if len(self.grasp_reference_offset_xyz) != 3:
            self.grasp_reference_offset_xyz = [0.0, 0.0, 0.0]
        self.grasp_rpy_deg = [
            float(v) for v in self.get_parameter("grasp_rpy_deg").value
        ]
        self.virtual_grasp_wait_sec: float = float(
            self.get_parameter("virtual_grasp_wait_sec").value
        )
        self.virtual_release_wait_sec: float = float(
            self.get_parameter("virtual_release_wait_sec").value
        )
        self.max_gripper_retries: int = int(
            self.get_parameter("max_gripper_retries").value
        )

        self.max_grasp_retries: int = int(
            self.get_parameter("max_grasp_retries").value
        )
        self.max_declutter_per_target: int = int(
            self.get_parameter("max_declutter_per_target").value
        )
        self.max_place_retries: int = int(
            self.get_parameter("max_place_retries").value
        )

        place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        place_rpy = [float(v) for v in self.get_parameter("place_rpy_deg").value]
        self.place_pose = self.build_pose(
            place_xyz[0], place_xyz[1], place_xyz[2],
            place_rpy[0], place_rpy[1], place_rpy[2]
        )
        place_slots_xyz = [
            float(v) for v in self.get_parameter("place_slots_xyz").value
        ]
        if len(place_slots_xyz) % 3 != 0:
            self.get_logger().warn(
                "place_slots_xyz length must be a multiple of 3; "
                "falling back to place_xyz for all targets."
            )
            place_slots_xyz = []
        self.place_poses = [
            self.build_pose(
                place_slots_xyz[i],
                place_slots_xyz[i + 1],
                place_slots_xyz[i + 2],
                place_rpy[0],
                place_rpy[1],
                place_rpy[2],
            )
            for i in range(0, len(place_slots_xyz), 3)
        ]

        declutter_xyz = [
            float(v) for v in self.get_parameter("declutter_temp_xyz").value
        ]
        declutter_rpy = [
            float(v) for v in self.get_parameter("declutter_temp_rpy_deg").value
        ]
        self.declutter_temp_pose = self.build_pose(
            declutter_xyz[0], declutter_xyz[1], declutter_xyz[2],
            declutter_rpy[0], declutter_rpy[1], declutter_rpy[2]
        )
        self.carry_lift_enabled: bool = bool(
            self.get_parameter("carry_lift_enabled").value
        )
        self.carry_lift_z: float = float(
            self.get_parameter("carry_lift_z").value
        )

        self.gripper_close_position: float = float(
            self.get_parameter("gripper_close_position").value
        )
        self.gripper_open_position: float = float(
            self.get_parameter("gripper_open_position").value
        )
        self.gripper_approach_position: float = float(
            self.get_parameter("gripper_approach_position").value
        )
        self.gripper_approach_wait_sec: float = float(
            self.get_parameter("gripper_approach_wait_sec").value
        )

        # ==================================================
        # Runtime state
        # ==================================================
        self.state: str = "INIT"
        self.state_enter_time: float = time.monotonic()

        self.current_index: int = 0
        self.expected_target_name: str = ""
        self.original_target_before_declutter: Optional[str] = None

        self.declutter_attempts = {
            name: 0 for name in self.target_sequence
        }
        self.declutter_candidates: List[str] = []
        self.declutter_candidate_index: int = 0
        self.current_declutter_candidate: Optional[str] = None

        self.place_retry_count: int = 0
        self.grasp_retry_count: int = 0
        self.gripper_retry_count: int = 0

        # Latest inputs
        self.current_target_visible: bool = False
        self.current_target_name_echo: str = ""
        self.latest_target_point_world: Optional[PointStamped] = None
        self.grasp_target_point_world: Optional[PointStamped] = None

        self.pregrasp_running: bool = False
        self.pregrasp_done: bool = False
        self.pregrasp_success: bool = False

        self.goal_pose_running: bool = False
        self.goal_pose_done: bool = False
        self.goal_pose_success: bool = False

        # 夹爪闭环反馈 (来自 /task/gripper_done + /task/gripper_success)
        self.gripper_done: bool = False
        self.gripper_success: bool = False

        # ==================================================
        # Subscribers
        # ==================================================
        self.create_subscription(
            Bool,
            "/perception/current_target_visible",
            self.current_target_visible_callback,
            10,
        )
        self.create_subscription(
            String,
            "/perception/current_target_name_echo",
            self.current_target_name_echo_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/perception/current_target_point_world",
            self.current_target_point_world_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/task/pregrasp_running",
            self.pregrasp_running_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/pregrasp_done",
            self.pregrasp_done_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/pregrasp_success",
            self.pregrasp_success_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/task/goal_pose_running",
            self.goal_pose_running_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/goal_pose_done",
            self.goal_pose_done_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/goal_pose_success",
            self.goal_pose_success_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/gripper_done",
            self.gripper_done_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/task/gripper_success",
            self.gripper_success_callback,
            10,
        )

        # ==================================================
        # Publishers
        # ==================================================
        self.current_target_name_pub = self.create_publisher(
            String,
            "/task/current_target_name",
            10,
        )
        self.enable_pregrasp_pub = self.create_publisher(
            Bool,
            "/task/enable_pregrasp",
            10,
        )
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            "/task/goal_pose_world",
            10,
        )
        self.execute_goal_pose_pub = self.create_publisher(
            Bool,
            "/task/execute_goal_pose",
            10,
        )
        self.gripper_command_pub = self.create_publisher(
            Float64,
            "/task/gripper_command",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "/task/state",
            10,
        )
        self.status_text_pub = self.create_publisher(
            String,
            "/task/status_text",
            10,
        )
        self.current_target_index_pub = self.create_publisher(
            Int32,
            "/task/current_target_index",
            10,
        )

        self.timer = self.create_timer(self.loop_period_sec, self.step)

        self.get_logger().info(
            "task_state_machine_node started. "
            f"target_sequence={self.target_sequence}, "
            f"target_frame={self.target_frame}, "
            f"grasp_height_offset={self.grasp_height_offset:.4f}, "
            f"grasp_height_offsets="
            f"{[round(v, 4) for v in self.grasp_height_offsets]}, "
            f"place_slots={len(self.place_poses)}"
        )
        self.publish_state()
        self.publish_status("Initialized")

    # ==================================================
    # Callbacks
    # ==================================================
    def current_target_visible_callback(self, msg: Bool):
        self.current_target_visible = bool(msg.data)

    def current_target_name_echo_callback(self, msg: String):
        self.current_target_name_echo = msg.data.strip()

    def current_target_point_world_callback(self, msg: PointStamped):
        self.latest_target_point_world = msg

    def pregrasp_running_callback(self, msg: Bool):
        self.pregrasp_running = bool(msg.data)

    def pregrasp_done_callback(self, msg: Bool):
        self.pregrasp_done = bool(msg.data)

    def pregrasp_success_callback(self, msg: Bool):
        self.pregrasp_success = bool(msg.data)

    def goal_pose_running_callback(self, msg: Bool):
        self.goal_pose_running = bool(msg.data)

    def goal_pose_done_callback(self, msg: Bool):
        self.goal_pose_done = bool(msg.data)

    def goal_pose_success_callback(self, msg: Bool):
        self.goal_pose_success = bool(msg.data)

    def gripper_done_callback(self, msg: Bool):
        self.gripper_done = bool(msg.data)

    def gripper_success_callback(self, msg: Bool):
        self.gripper_success = bool(msg.data)

    # ==================================================
    # Helpers
    # ==================================================
    def build_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> PoseStamped:
        qx, qy, qz, qw = euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg)

        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def set_state(self, new_state: str, reason: str = ""):
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        self.state_enter_time = time.monotonic()

        if reason:
            self.get_logger().info(f"STATE: {old_state} -> {new_state} | {reason}")
            self.publish_status(f"{old_state} -> {new_state} | {reason}")
        else:
            self.get_logger().info(f"STATE: {old_state} -> {new_state}")
            self.publish_status(f"{old_state} -> {new_state}")

        self.publish_state()

        # FR-4.1 / FR-5.2: 夹爪状态切入时 reset 反馈 flags + 发开合指令,
        # 然后由 step() 等 /task/gripper_done 推进下一态。
        if new_state == "VIRTUAL_GRASP":
            self.gripper_retry_count = 0
            self.reset_gripper_flags()
            self.publish_gripper_command(self.gripper_close_position)
        elif new_state == "GRIPPER_APPROACH":
            # 下爪前先把夹爪从全开收到 approach 开度, 减少 pad 外缘横向占用,
            # 避免下落时碰到邻居物体。
            self.gripper_retry_count = 0
            self.reset_gripper_flags()
            self.publish_gripper_command(self.gripper_approach_position)
        elif new_state in ("VIRTUAL_RELEASE", "DECLUTTER_VIRTUAL_RELEASE"):
            self.gripper_retry_count = 0
            self.reset_gripper_flags()
            self.publish_gripper_command(self.gripper_open_position)

    def time_in_state(self) -> float:
        return time.monotonic() - self.state_enter_time

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_text_pub.publish(msg)

    def publish_current_target_index(self):
        msg = Int32()
        msg.data = int(self.current_index)
        self.current_target_index_pub.publish(msg)

    def publish_current_target_name(self, target_name: str):
        self.expected_target_name = target_name
        self.current_target_visible = False  # 本地先清掉，等 perception 刷新
        self.latest_target_point_world = None
        self.grasp_target_point_world = None
        msg = String()
        msg.data = target_name
        self.current_target_name_pub.publish(msg)
        self.get_logger().info(f"Publish current target: {target_name}")

    def publish_current_target_name_heartbeat(self):
        if not self.expected_target_name:
            return

        msg = String()
        msg.data = self.expected_target_name
        self.current_target_name_pub.publish(msg)

    def publish_gripper_command(self, position: float):
        msg = Float64()
        msg.data = float(position)
        self.gripper_command_pub.publish(msg)
        self.get_logger().info(
            f"Gripper command: {position:.3f} "
            f"({'close' if position > 0.02 else 'open'})"
        )

    def retry_gripper_command(self, position: float, label: str) -> bool:
        if self.gripper_retry_count >= self.max_gripper_retries:
            return False

        self.gripper_retry_count += 1
        self.reset_gripper_flags()
        self.state_enter_time = time.monotonic()
        self.publish_gripper_command(position)

        text = (
            f"Retry gripper {label} "
            f"{self.gripper_retry_count}/{self.max_gripper_retries}"
        )
        self.get_logger().warn(text)
        self.publish_status(text)
        return True

    def perception_aligned_with_expected_target(self) -> bool:
        return self.current_target_name_echo == self.expected_target_name

    def reset_pregrasp_flags(self):
        self.pregrasp_running = False
        self.pregrasp_done = False
        self.pregrasp_success = False

    def reset_goal_pose_flags(self):
        self.goal_pose_running = False
        self.goal_pose_done = False
        self.goal_pose_success = False

    def reset_gripper_flags(self):
        self.gripper_done = False
        self.gripper_success = False

    def trigger_pregrasp(self):
        self.reset_pregrasp_flags()
        msg = Bool()
        msg.data = True
        self.enable_pregrasp_pub.publish(msg)
        self.get_logger().info("Triggered pregrasp executor")

    def pregrasp_executor_ready(self) -> bool:
        return self.enable_pregrasp_pub.get_subscription_count() > 0

    def goal_pose_executor_ready(self) -> bool:
        return (
            self.goal_pose_pub.get_subscription_count() > 0
            and self.execute_goal_pose_pub.get_subscription_count() > 0
        )

    def wait_for_executor_match(self, label: str, ready: bool) -> bool:
        if ready:
            return True

        if self.time_in_state() < self.executor_match_timeout_sec:
            return False

        self.get_logger().warn(
            f"{label} executor subscription not matched after "
            f"{self.executor_match_timeout_sec:.1f}s; publishing trigger anyway"
        )
        return True

    def trigger_goal_pose(self, pose: PoseStamped):
        self.reset_goal_pose_flags()

        pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose_pub.publish(pose)

        msg = Bool()
        msg.data = True
        self.execute_goal_pose_pub.publish(msg)

        self.get_logger().info(
            "Triggered pose executor: "
            f"({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, "
            f"{pose.pose.position.z:.3f})"
        )

    def get_current_target_name(self) -> Optional[str]:
        if 0 <= self.current_index < len(self.target_sequence):
            return self.target_sequence[self.current_index]
        return None

    def get_current_grasp_height_offset(self) -> float:
        if 0 <= self.current_index < len(self.grasp_height_offsets):
            return self.grasp_height_offsets[self.current_index]
        return self.grasp_height_offset

    def get_current_place_pose(self) -> PoseStamped:
        if 0 <= self.current_index < len(self.place_poses):
            return self.place_poses[self.current_index]
        return self.place_pose

    def build_declutter_candidates(self, original_target: str) -> List[str]:
        return [name for name in self.target_sequence if name != original_target]

    def build_gripper_center_xyz(
        self,
        reference_x: float,
        reference_y: float,
        reference_z: float,
    ) -> List[float]:
        offset_world = rotate_vector_by_rpy_deg(
            self.grasp_reference_offset_xyz,
            self.grasp_rpy_deg[0],
            self.grasp_rpy_deg[1],
            self.grasp_rpy_deg[2],
        )
        return [
            reference_x - offset_world[0],
            reference_y - offset_world[1],
            reference_z - offset_world[2],
        ]

    def build_current_grasp_pose(self) -> Optional[PoseStamped]:
        point_msg = self.grasp_target_point_world or self.latest_target_point_world
        if point_msg is None:
            return None

        p = point_msg.point
        height_offset = self.get_current_grasp_height_offset()
        x, y, z = self.build_gripper_center_xyz(
            p.x + self.grasp_position_offset_xyz[0],
            p.y + self.grasp_position_offset_xyz[1],
            p.z + height_offset + self.grasp_position_offset_xyz[2],
        )
        self.get_logger().info(
            f"Build grasp pose target={self.get_current_target_name()}, "
            f"detected_z={p.z:.3f}, height_offset={height_offset:.4f}, "
            f"gripper_center=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"grasp_rpy=({self.grasp_rpy_deg[0]:.0f},"
            f"{self.grasp_rpy_deg[1]:.0f},{self.grasp_rpy_deg[2]:.0f})"
        )
        return self.build_pose(
            x,
            y,
            z,
            self.grasp_rpy_deg[0],
            self.grasp_rpy_deg[1],
            self.grasp_rpy_deg[2],
        )

    def build_carry_lift_pose(self) -> Optional[PoseStamped]:
        point_msg = self.grasp_target_point_world or self.latest_target_point_world
        if point_msg is None:
            return None

        p = point_msg.point
        height_offset = self.get_current_grasp_height_offset()
        z = max(
            float(self.carry_lift_z),
            float(p.z) + height_offset + self.grasp_position_offset_xyz[2],
        )
        x, y, z = self.build_gripper_center_xyz(
            p.x + self.grasp_position_offset_xyz[0],
            p.y + self.grasp_position_offset_xyz[1],
            z,
        )
        return self.build_pose(
            x,
            y,
            z,
            self.grasp_rpy_deg[0],
            self.grasp_rpy_deg[1],
            self.grasp_rpy_deg[2],
        )

    # ==================================================
    # Main step
    # ==================================================
    def step(self):
        self.publish_state()
        self.publish_current_target_index()
        if self.state not in (
            "INIT",
            "SET_TARGET",
            "NEXT_TARGET",
            "RESTORE_ORIGINAL_TARGET",
            "FINISHED",
        ):
            self.publish_current_target_name_heartbeat()

        if self.state == "INIT":
            if len(self.target_sequence) == 0:
                self.set_state("FINISHED", "Empty target_sequence")
                return

            self.current_index = 0
            self.place_retry_count = 0
            self.set_state("SET_TARGET", "Start sequence")
            return

        if self.state == "SET_TARGET":
            target_name = self.get_current_target_name()
            if target_name is None:
                self.set_state("FINISHED", "All targets processed")
                return

            self.publish_current_target_name(target_name)
            self.place_retry_count = 0
            self.grasp_retry_count = 0
            self.set_state("WAIT_TARGET_SETTLE", f"Target={target_name}")
            return

        if self.state == "WAIT_TARGET_SETTLE":
            if self.time_in_state() >= self.target_settle_sec:
                self.set_state(
                    "WAIT_TARGET_VISIBLE",
                    f"Wait visible for target={self.expected_target_name}",
                )
            return

        if self.state == "WAIT_TARGET_VISIBLE":
            if (
                self.perception_aligned_with_expected_target()
                and self.current_target_visible
            ):
                self.grasp_target_point_world = self.latest_target_point_world
                self.set_state(
                    "PLAN_PREGRASP",
                    f"Target visible: {self.expected_target_name}",
                )
                return

            if self.time_in_state() >= self.target_visible_timeout_sec:
                current_target = self.get_current_target_name()
                if current_target is None:
                    self.set_state("FINISHED", "Current target is None")
                    return

                if self.declutter_attempts[current_target] < self.max_declutter_per_target:
                    self.declutter_attempts[current_target] += 1
                    self.set_state(
                        "DECLUTTER_SELECT",
                        f"Target invisible timeout, declutter attempt "
                        f"{self.declutter_attempts[current_target]}/"
                        f"{self.max_declutter_per_target}",
                    )
                else:
                    self.set_state(
                        "NEXT_TARGET",
                        f"Target invisible and declutter exhausted for {current_target}",
                    )
            return

        if self.state == "PLAN_PREGRASP":
            if not self.wait_for_executor_match(
                "Pregrasp",
                self.pregrasp_executor_ready(),
            ):
                return
            self.trigger_pregrasp()
            self.set_state("WAIT_PREGRASP_RESULT", "Waiting pregrasp result")
            return

        if self.state == "WAIT_PREGRASP_RESULT":
            if self.pregrasp_done:
                if self.pregrasp_success:
                    self.set_state("VERIFY_PREGRASP", "Pregrasp success")
                else:
                    current_target = self.get_current_target_name()
                    if current_target is not None and \
                       self.declutter_attempts[current_target] < self.max_declutter_per_target:
                        self.declutter_attempts[current_target] += 1
                        self.set_state(
                            "DECLUTTER_SELECT",
                            f"Pregrasp failed, declutter attempt "
                            f"{self.declutter_attempts[current_target]}/"
                            f"{self.max_declutter_per_target}",
                        )
                    else:
                        self.set_state("NEXT_TARGET", "Pregrasp failed and no declutter left")
            return

        if self.state == "VERIFY_PREGRASP":
            if self.time_in_state() >= self.verify_pregrasp_wait_sec:
                self.set_state(
                    "GRIPPER_APPROACH",
                    "Pregrasp verified, narrow gripper before descent",
                )
            return

        if self.state == "GRIPPER_APPROACH":
            if self.gripper_done:
                self.set_state(
                    "MOVE_TO_GRASP",
                    f"Gripper narrowed to approach={self.gripper_approach_position:.3f}",
                )
            elif self.time_in_state() >= self.gripper_approach_wait_sec:
                self.get_logger().warn(
                    "Gripper approach command timed out; "
                    "proceeding to MOVE_TO_GRASP anyway"
                )
                self.set_state(
                    "MOVE_TO_GRASP",
                    "Gripper approach timeout, continue to grasp",
                )
            return

        if self.state == "MOVE_TO_GRASP":
            if not self.wait_for_executor_match(
                "Goal pose",
                self.goal_pose_executor_ready(),
            ):
                return
            grasp_pose = self.build_current_grasp_pose()
            if grasp_pose is None:
                self.set_state("NEXT_TARGET", "No target point for grasp pose")
                return

            self.trigger_goal_pose(grasp_pose)
            self.set_state("WAIT_GRASP_RESULT", "Waiting grasp pose result")
            return

        if self.state == "WAIT_GRASP_RESULT":
            if self.goal_pose_done:
                if self.goal_pose_success:
                    self.set_state("VIRTUAL_GRASP", "Grasp pose success")
                else:
                    if self.grasp_retry_count < self.max_grasp_retries:
                        self.grasp_retry_count += 1
                        self.set_state(
                            "MOVE_TO_GRASP",
                            f"Retry grasp pose "
                            f"{self.grasp_retry_count}/{self.max_grasp_retries}",
                        )
                    else:
                        self.set_state("NEXT_TARGET", "Grasp pose failed")
            return

        if self.state == "VIRTUAL_GRASP":
            if self.gripper_done:
                if self.gripper_success:
                    if self.carry_lift_enabled:
                        self.set_state("LIFT_AFTER_GRASP", "Gripper closed (success)")
                    else:
                        self.set_state("MOVE_TO_PLACE_HOVER", "Gripper closed (success)")
                else:
                    if self.retry_gripper_command(
                        self.gripper_close_position,
                        "close",
                    ):
                        return
                    self.set_state(
                        "FINISHED",
                        "Gripper close action failed after retries; stopping sequence",
                    )
            elif self.time_in_state() >= self.virtual_grasp_wait_sec:
                if self.retry_gripper_command(
                    self.gripper_close_position,
                    "close timeout",
                ):
                    return
                self.set_state(
                    "FINISHED",
                    f"Gripper close timeout after retries "
                    f"({self.virtual_grasp_wait_sec}s); stopping sequence",
                )
            return

        if self.state == "LIFT_AFTER_GRASP":
            if not self.wait_for_executor_match(
                "Goal pose",
                self.goal_pose_executor_ready(),
            ):
                return
            lift_pose = self.build_carry_lift_pose()
            if lift_pose is None:
                self.set_state(
                    "MOVE_TO_PLACE_HOVER",
                    "No target point for lift pose; continue to place",
                )
                return

            self.trigger_goal_pose(lift_pose)
            self.set_state("WAIT_LIFT_RESULT", "Waiting carry lift result")
            return

        if self.state == "WAIT_LIFT_RESULT":
            if self.goal_pose_done:
                if self.goal_pose_success:
                    self.set_state("MOVE_TO_PLACE_HOVER", "Carry lift success")
                else:
                    self.set_state(
                        "MOVE_TO_PLACE_HOVER",
                        "Carry lift failed; continue to place hover",
                    )
            return

        if self.state == "MOVE_TO_PLACE_HOVER":
            if not self.wait_for_executor_match(
                "Goal pose",
                self.goal_pose_executor_ready(),
            ):
                return
            self.trigger_goal_pose(self.get_current_place_pose())
            self.set_state("WAIT_PLACE_RESULT", "Waiting place hover result")
            return

        if self.state == "WAIT_PLACE_RESULT":
            if self.goal_pose_done:
                if self.goal_pose_success:
                    self.set_state("VIRTUAL_RELEASE", "Place hover success")
                else:
                    if self.place_retry_count < self.max_place_retries:
                        self.place_retry_count += 1
                        self.set_state(
                            "MOVE_TO_PLACE_HOVER",
                            f"Retry place hover {self.place_retry_count}/{self.max_place_retries}",
                        )
                    else:
                        self.set_state("NEXT_TARGET", "Place hover failed")
            return

        if self.state == "VIRTUAL_RELEASE":
            if self.gripper_done:
                if self.gripper_success:
                    self.set_state("NEXT_TARGET", "Gripper opened (success)")
                else:
                    if self.retry_gripper_command(
                        self.gripper_open_position,
                        "open",
                    ):
                        return
                    self.set_state(
                        "FINISHED",
                        "Gripper open action failed after retries; stopping sequence",
                    )
            elif self.time_in_state() >= self.virtual_release_wait_sec:
                if self.retry_gripper_command(
                    self.gripper_open_position,
                    "open timeout",
                ):
                    return
                self.set_state(
                    "FINISHED",
                    f"Gripper open timeout after retries "
                    f"({self.virtual_release_wait_sec}s); stopping sequence",
                )
            return

        if self.state == "NEXT_TARGET":
            self.current_index += 1
            if self.current_index >= len(self.target_sequence):
                self.set_state("FINISHED", "Sequence completed")
            else:
                self.set_state("SET_TARGET", "Move to next target")
            return

        if self.state == "DECLUTTER_SELECT":
            original_target = self.get_current_target_name()
            if original_target is None:
                self.set_state("FINISHED", "Declutter but current target is None")
                return

            self.original_target_before_declutter = original_target
            self.declutter_candidates = self.build_declutter_candidates(original_target)
            self.declutter_candidate_index = 0
            self.current_declutter_candidate = None

            self.set_state(
                "SET_DECLUTTER_CANDIDATE",
                f"Original target={original_target}, candidates={self.declutter_candidates}",
            )
            return

        if self.state == "SET_DECLUTTER_CANDIDATE":
            if self.declutter_candidate_index >= len(self.declutter_candidates):
                self.set_state(
                    "RESTORE_ORIGINAL_TARGET",
                    "No visible declutter candidate found, restore original target",
                )
                return

            self.current_declutter_candidate = self.declutter_candidates[
                self.declutter_candidate_index
            ]
            self.publish_current_target_name(self.current_declutter_candidate)
            self.set_state(
                "WAIT_DECLUTTER_SETTLE",
                f"Check declutter candidate={self.current_declutter_candidate}",
            )
            return

        if self.state == "WAIT_DECLUTTER_SETTLE":
            if self.time_in_state() >= self.target_settle_sec:
                self.set_state(
                    "WAIT_DECLUTTER_VISIBLE",
                    f"Wait visible for declutter candidate={self.expected_target_name}",
                )
            return

        if self.state == "WAIT_DECLUTTER_VISIBLE":
            if (
                self.perception_aligned_with_expected_target()
                and self.current_target_visible
            ):
                self.set_state(
                    "PLAN_DECLUTTER_PREGRASP",
                    f"Declutter candidate visible={self.expected_target_name}",
                )
                return

            if self.time_in_state() >= self.declutter_candidate_visible_timeout_sec:
                self.declutter_candidate_index += 1
                self.set_state(
                    "SET_DECLUTTER_CANDIDATE",
                    "Current declutter candidate not visible, try next",
                )
            return

        if self.state == "PLAN_DECLUTTER_PREGRASP":
            if not self.wait_for_executor_match(
                "Pregrasp",
                self.pregrasp_executor_ready(),
            ):
                return
            self.trigger_pregrasp()
            self.set_state(
                "WAIT_DECLUTTER_PREGRASP_RESULT",
                "Waiting declutter pregrasp result",
            )
            return

        if self.state == "WAIT_DECLUTTER_PREGRASP_RESULT":
            if self.pregrasp_done:
                if self.pregrasp_success:
                    self.set_state(
                        "MOVE_DECLUTTER_TEMP_PLACE",
                        "Declutter pregrasp success",
                    )
                else:
                    self.declutter_candidate_index += 1
                    self.set_state(
                        "SET_DECLUTTER_CANDIDATE",
                        "Declutter pregrasp failed, try next candidate",
                    )
            return

        if self.state == "MOVE_DECLUTTER_TEMP_PLACE":
            if not self.wait_for_executor_match(
                "Goal pose",
                self.goal_pose_executor_ready(),
            ):
                return
            self.trigger_goal_pose(self.declutter_temp_pose)
            self.set_state(
                "WAIT_DECLUTTER_PLACE_RESULT",
                "Waiting declutter temp place result",
            )
            return

        if self.state == "WAIT_DECLUTTER_PLACE_RESULT":
            if self.goal_pose_done:
                if self.goal_pose_success:
                    self.set_state(
                        "DECLUTTER_VIRTUAL_RELEASE",
                        "Declutter temp place success",
                    )
                else:
                    self.declutter_candidate_index += 1
                    self.set_state(
                        "SET_DECLUTTER_CANDIDATE",
                        "Declutter temp place failed, try next candidate",
                    )
            return

        if self.state == "DECLUTTER_VIRTUAL_RELEASE":
            if self.gripper_done:
                if self.gripper_success:
                    self.set_state(
                        "RESTORE_ORIGINAL_TARGET",
                        "Declutter gripper opened (success)",
                    )
                else:
                    if self.retry_gripper_command(
                        self.gripper_open_position,
                        "declutter open",
                    ):
                        return
                    self.set_state(
                        "FINISHED",
                        "Declutter gripper open action failed after retries; "
                        "stopping sequence",
                    )
            elif self.time_in_state() >= self.virtual_release_wait_sec:
                if self.retry_gripper_command(
                    self.gripper_open_position,
                    "declutter open timeout",
                ):
                    return
                self.set_state(
                    "FINISHED",
                    f"Declutter gripper open timeout after retries "
                    f"({self.virtual_release_wait_sec}s); stopping sequence",
                )
            return

        if self.state == "RESTORE_ORIGINAL_TARGET":
            if self.original_target_before_declutter is None:
                self.set_state("NEXT_TARGET", "No original target recorded")
                return

            self.publish_current_target_name(self.original_target_before_declutter)
            self.set_state(
                "WAIT_TARGET_SETTLE",
                f"Restore original target={self.original_target_before_declutter}",
            )
            return

        if self.state == "FINISHED":
            return

        self.get_logger().warn(f"Unknown state: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskStateMachineNode()
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
