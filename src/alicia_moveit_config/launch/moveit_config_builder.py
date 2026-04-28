"""
moveit_config_builder.py — Alicia MoveIt2 配置构建器
用于 sim_demo.launch.py 调用，使用 alicia_moveit_config 包。
"""

import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_config():
    """
    构建并返回 Alicia-D 6-DOF + 原始夹爪的 MoveIt2 配置。
    使用 alicia_moveit_config 包，硬件接口为 gz_ros2_control。
    """
    pkg_name = 'alicia_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_path = os.path.join(
        pkg_share, 'config', 'Alicia_D_v5_6_gripper_100mm.urdf.xacro'
    )
    srdf_path = os.path.join(
        pkg_share, 'config', 'Alicia_D_v5_6_gripper_100mm.srdf'
    )

    moveit_config = (
        MoveItConfigsBuilder('Alicia_D_v5_6_gripper_100mm', package_name=pkg_name)
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, 'config', 'kinematics.yaml')
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, 'config', 'joint_limits.yaml')
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_share, 'config', 'moveit_controllers.yaml')
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    return moveit_config
