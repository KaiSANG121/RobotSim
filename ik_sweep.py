#!/usr/bin/env python3
"""
IK feasibility sweep — runs on the Jetson with ROS 2 Humble.
Tests grasp_rpy_deg=(180, pitch, 0) for pitch in [90, 80, ..., 0]
for all three target positions.
"""
import math
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


TARGETS = [
    ("red_cylinder", -0.299, 0.255, 0.310),
    ("yellow_box",   -0.357, 0.104, 0.300),
    ("brown_cube",   -0.238, 0.103, 0.265),
]


def main():
    rclpy.init()
    node = Node("ik_sweep")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("compute_ik service not available")
        rclpy.shutdown()
        sys.exit(1)

    print(f"{'pitch':>6}  {'red_cylinder':>14}  {'yellow_box':>14}  {'brown_cube':>14}")
    print("-" * 60)

    for pitch in [90, 80, 70, 60, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0]:
        qx, qy, qz, qw = rpy_to_quat(180, pitch, 0)
        results = []
        for name, px, py, pz in TARGETS:
            req = GetPositionIK.Request()
            req.ik_request.group_name = "Alicia"
            req.ik_request.robot_state.joint_state.name = [
                "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
            ]
            req.ik_request.robot_state.joint_state.position = [0.0] * 6
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = pz
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            req.ik_request.pose_stamped = ps
            req.ik_request.timeout = Duration(sec=5, nanosec=0)

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            resp = future.result()
            if resp is None:
                results.append("TIMEOUT")
            elif resp.error_code.val == MoveItErrorCodes.SUCCESS:
                results.append("OK")
            else:
                results.append(f"FAIL({resp.error_code.val})")

        print(f"{pitch:>5}°  {results[0]:>14}  {results[1]:>14}  {results[2]:>14}")
        sys.stdout.flush()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
