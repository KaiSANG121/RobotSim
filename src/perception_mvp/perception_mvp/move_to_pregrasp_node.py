"""
move_to_pregrasp_node.py — 执行层节点（重写版 v2.1）

修复了旧版的 3 个致命问题:
  1. Topic 匹配: 订阅 /perception/current_target_pregrasp_world (PointStamped)
                 ← 旧版错误订阅 /perception/current_pregrasp_pose_world (PoseStamped)
  2. 类型转换: PointStamped → PoseStamped (补全抓取朝向 roll=180°, 使末端俯视抓取)
  3. 事件驱动: 响应 /task/enable_pregrasp (Bool) 和 /task/execute_goal_pose (Bool)
               ← 旧版独立轮询，不响应状态机指令

新增:
  - 6 个反馈 Topic (pregrasp_running/done/success, goal_pose_running/done/success)
  - 夹爪控制: 响应 /task/gripper_command (Float64)
  - 多线程安全: ReentrantCallbackGroup + MultiThreadedExecutor
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from pymoveit2 import MoveIt2


def point_stamped_to_pose_stamped(
    point: PointStamped,
    roll_deg: float = 180.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> PoseStamped:
    """
    将 PointStamped 转换为 PoseStamped，补全末端抓取姿态。
    默认 roll=180° 表示末端朝下（俯视抓取）。
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    pose = PoseStamped()
    pose.header = point.header
    pose.pose.position.x = point.point.x
    pose.pose.position.y = point.point.y
    pose.pose.position.z = point.point.z
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose


class MoveToPregraspNode(Node):
    def __init__(self):
        super().__init__('move_to_pregrasp_node')

        # ===== Robot / MoveIt parameters =====
        self.declare_parameter('joint_names', [
            'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'
        ])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'gripper_center')
        self.declare_parameter('group_name', 'Alicia')
        self.declare_parameter('gripper_group_name', 'Gripper')
        self.declare_parameter('gripper_joint_name', 'Gripper')

        # ===== Motion parameters =====
        self.declare_parameter('use_move_group_action', True)
        self.declare_parameter('cartesian', False)
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.05)
        self.declare_parameter('max_velocity', 0.10)
        self.declare_parameter('max_acceleration', 0.10)

        # ===== Grasp orientation =====
        self.declare_parameter('grasp_roll_deg', 180.0)
        self.declare_parameter('grasp_pitch_deg', 0.0)
        self.declare_parameter('grasp_yaw_deg', 0.0)

        # ===== Startup =====
        self.declare_parameter('startup_wait_sec', 5.0)

        # Read params
        joint_names = self.get_parameter('joint_names').value
        base_link_name = self.get_parameter('base_link_name').value
        end_effector_name = self.get_parameter('end_effector_name').value
        group_name = self.get_parameter('group_name').value

        self.gripper_joint_name = str(
            self.get_parameter('gripper_joint_name').value
        )
        self.use_move_group_action = bool(
            self.get_parameter('use_move_group_action').value
        )
        self.cartesian = bool(self.get_parameter('cartesian').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )
        self.orientation_tolerance = float(
            self.get_parameter('orientation_tolerance').value
        )
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value
        )
        self.grasp_roll_deg = float(
            self.get_parameter('grasp_roll_deg').value
        )
        self.grasp_pitch_deg = float(
            self.get_parameter('grasp_pitch_deg').value
        )
        self.grasp_yaw_deg = float(
            self.get_parameter('grasp_yaw_deg').value
        )
        self.startup_wait_sec = float(
            self.get_parameter('startup_wait_sec').value
        )

        self.callback_group = ReentrantCallbackGroup()

        # MoveIt2 接口
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            use_move_group_action=self.use_move_group_action,
            ignore_new_calls_while_executing=True,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        # ===== 内部状态 =====
        self._lock = threading.Lock()

        self.latest_pregrasp_point: Optional[PointStamped] = None
        self.latest_goal_pose: Optional[PoseStamped] = None

        self.pregrasp_triggered: bool = False
        self.goal_pose_triggered: bool = False
        self.gripper_command: Optional[float] = None

        # ===== Subscribers =====

        # 感知层：预抓取位置（PointStamped，上方 0.10m）
        self.create_subscription(
            PointStamped,
            '/perception/current_target_pregrasp_world',
            self._pregrasp_point_callback,
            10,
            callback_group=self.callback_group,
        )

        # 决策层：使能 pregrasp 运动
        self.create_subscription(
            Bool,
            '/task/enable_pregrasp',
            self._enable_pregrasp_callback,
            10,
            callback_group=self.callback_group,
        )

        # 决策层：放置目标位姿
        self.create_subscription(
            PoseStamped,
            '/task/goal_pose_world',
            self._goal_pose_callback,
            10,
            callback_group=self.callback_group,
        )

        # 决策层：触发放置运动
        self.create_subscription(
            Bool,
            '/task/execute_goal_pose',
            self._execute_goal_pose_callback,
            10,
            callback_group=self.callback_group,
        )

        # 决策层：夹爪开合指令 (0.0=open, 0.05=closed)
        self.create_subscription(
            Float64,
            '/task/gripper_command',
            self._gripper_command_callback,
            10,
            callback_group=self.callback_group,
        )

        # ===== Publishers — 反馈 Topics =====

        self.pregrasp_running_pub = self.create_publisher(
            Bool, '/task/pregrasp_running', 10
        )
        self.pregrasp_done_pub = self.create_publisher(
            Bool, '/task/pregrasp_done', 10
        )
        self.pregrasp_success_pub = self.create_publisher(
            Bool, '/task/pregrasp_success', 10
        )
        self.goal_pose_running_pub = self.create_publisher(
            Bool, '/task/goal_pose_running', 10
        )
        self.goal_pose_done_pub = self.create_publisher(
            Bool, '/task/goal_pose_done', 10
        )
        self.goal_pose_success_pub = self.create_publisher(
            Bool, '/task/goal_pose_success', 10
        )

        # 夹爪控制器 action topic
        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory,
            '/Gripper_controller/joint_trajectory',
            10,
        )

        self.get_logger().info(
            'move_to_pregrasp_node started (v2.1 — event-driven). '
            f'group={group_name}, ee={end_effector_name}, '
            f'grasp_rpy=({self.grasp_roll_deg:.0f}°,'
            f'{self.grasp_pitch_deg:.0f}°,'
            f'{self.grasp_yaw_deg:.0f}°)'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _pregrasp_point_callback(self, msg: PointStamped):
        with self._lock:
            self.latest_pregrasp_point = msg

    def _enable_pregrasp_callback(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self.pregrasp_triggered = True
        self.get_logger().info('enable_pregrasp received → scheduling pregrasp motion')

    def _goal_pose_callback(self, msg: PoseStamped):
        with self._lock:
            self.latest_goal_pose = msg

    def _execute_goal_pose_callback(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self.goal_pose_triggered = True
        self.get_logger().info('execute_goal_pose received → scheduling place motion')

    def _gripper_command_callback(self, msg: Float64):
        with self._lock:
            self.gripper_command = float(msg.data)
        self.get_logger().info(
            f'gripper_command received: {msg.data:.3f} '
            f'({"close" if msg.data > 0.02 else "open"})'
        )

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _pub_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _publish_pregrasp_running(self, v: bool):
        self._pub_bool(self.pregrasp_running_pub, v)

    def _publish_pregrasp_done(self, success: bool):
        self._pub_bool(self.pregrasp_done_pub, True)
        self._pub_bool(self.pregrasp_success_pub, success)
        self._pub_bool(self.pregrasp_running_pub, False)

    def _publish_goal_pose_running(self, v: bool):
        self._pub_bool(self.goal_pose_running_pub, v)

    def _publish_goal_pose_done(self, success: bool):
        self._pub_bool(self.goal_pose_done_pub, True)
        self._pub_bool(self.goal_pose_success_pub, success)
        self._pub_bool(self.goal_pose_running_pub, False)

    # ------------------------------------------------------------------
    # Gripper control
    # ------------------------------------------------------------------

    def _send_gripper_position(self, position: float):
        """
        通过 JointTrajectory 发送夹爪位置指令。
        position: 0.0 = open, 0.05 = closed
        """
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self.gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)

        traj.points = [point]
        self.gripper_traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent gripper position: {position:.3f} '
            f'({"close" if position > 0.02 else "open"})'
        )

    # ------------------------------------------------------------------
    # Motion execution
    # ------------------------------------------------------------------

    def _execute_pregrasp(self) -> bool:
        """
        执行 pregrasp 运动：
        将 PointStamped 转为 PoseStamped（补全俯视抓取姿态），
        然后调用 MoveIt2 move_to_pose。
        """
        with self._lock:
            point = self.latest_pregrasp_point

        if point is None:
            self.get_logger().warn('No pregrasp point available, skip.')
            return False

        # PointStamped → PoseStamped（roll=180° 表示末端朝下）
        pose = point_stamped_to_pose_stamped(
            point,
            roll_deg=self.grasp_roll_deg,
            pitch_deg=self.grasp_pitch_deg,
            yaw_deg=self.grasp_yaw_deg,
        )

        position = [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        ]
        quat_xyzw = [
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        ]
        frame_id = pose.header.frame_id or self.target_frame

        self.get_logger().info(
            f'Executing pregrasp → position={[f"{v:.3f}" for v in position]}, '
            f'frame={frame_id}'
        )

        try:
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                frame_id=frame_id,
                tolerance_position=self.position_tolerance,
                tolerance_orientation=self.orientation_tolerance,
                cartesian=self.cartesian,
            )
            ok = self.moveit2.wait_until_executed()
            if not ok:
                self.get_logger().warn('Pregrasp motion aborted or failed.')
                return False

            self.get_logger().info('Pregrasp motion completed successfully.')
            return ok
        except Exception as e:
            self.get_logger().error(f'Pregrasp motion failed: {e}')
            return False

    def _execute_goal_pose(self) -> bool:
        """
        执行放置运动：移动到 goal_pose_world 指定的位姿。
        """
        with self._lock:
            pose = self.latest_goal_pose

        if pose is None:
            self.get_logger().warn('No goal pose available, skip.')
            return False

        position = [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        ]
        quat_xyzw = [
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        ]
        frame_id = pose.header.frame_id or self.target_frame

        self.get_logger().info(
            f'Executing goal pose → position={[f"{v:.3f}" for v in position]}, '
            f'frame={frame_id}'
        )

        try:
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                frame_id=frame_id,
                tolerance_position=self.position_tolerance,
                tolerance_orientation=self.orientation_tolerance,
                cartesian=self.cartesian,
            )
            ok = self.moveit2.wait_until_executed()
            if not ok:
                self.get_logger().warn('Goal pose motion aborted or failed.')
                return False

            self.get_logger().info('Goal pose motion completed successfully.')
            return ok
        except Exception as e:
            self.get_logger().error(f'Goal pose motion failed: {e}')
            return False

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        """
        事件驱动主循环（在独立线程中运行）：
        轮询 triggered 标志，触发时执行对应运动并发布反馈。
        """
        self.get_logger().info(
            f'Waiting {self.startup_wait_sec:.0f}s for MoveIt2 / controllers...'
        )
        time.sleep(self.startup_wait_sec)
        self.get_logger().info('move_to_pregrasp_node ready.')

        while rclpy.ok():
            # ---- 检查夹爪指令 ----
            with self._lock:
                gripper_cmd = self.gripper_command
                self.gripper_command = None

            if gripper_cmd is not None:
                self._send_gripper_position(gripper_cmd)

            # ---- 检查 pregrasp 触发 ----
            with self._lock:
                pregrasp_go = self.pregrasp_triggered
                if pregrasp_go:
                    self.pregrasp_triggered = False

            if pregrasp_go:
                self._publish_pregrasp_running(True)
                success = self._execute_pregrasp()
                self._publish_pregrasp_done(success)

            # ---- 检查 goal_pose 触发 ----
            with self._lock:
                goal_go = self.goal_pose_triggered
                if goal_go:
                    self.goal_pose_triggered = False

            if goal_go:
                self._publish_goal_pose_running(True)
                success = self._execute_goal_pose()
                self._publish_goal_pose_done(success)

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPregraspNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # 后台 spin（处理订阅回调 + MoveIt2 action/service）
    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    # 前台事件驱动主循环
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
