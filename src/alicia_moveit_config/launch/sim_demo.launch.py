"""
sim_demo.launch.py — Alicia-D 6-DOF 仿真系统统一启动文件

启动顺序:
  1. Gazebo Sim (bin_scene.sdf)
  2. robot_state_publisher
  3. controller spawner (joint_state_broadcaster + Alicia + Gripper)
  4. MoveIt move_group (等控制器就绪)
  5. ros_gz_bridge (相机数据桥接)
  6. static_transform_publisher (world -> rgbd_camera_frame)
  7. RViz (可选)

用法:
  ros2 launch alicia_moveit_config sim_demo.launch.py
  ros2 launch alicia_moveit_config sim_demo.launch.py use_rviz:=false
"""

import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_moveit_config


def world_file_for_render_engine(source_world_file, render_engine):
    """Create a temporary world file with the requested sensor render engine."""
    with open(source_world_file, 'r', encoding='utf-8') as f:
        world_xml = f.read()

    world_xml = world_xml.replace(
        '<render_engine>ogre</render_engine>',
        f'<render_engine>{render_engine}</render_engine>',
    )

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        suffix=f'_{render_engine}.sdf',
        prefix='robotsim_bin_scene_',
        delete=False,
    )
    with tmp:
        tmp.write(world_xml)
    return tmp.name


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    render_engine = LaunchConfiguration('render_engine').perform(context).lower()

    if render_engine == 'auto':
        if os.environ.get('LIBGL_ALWAYS_SOFTWARE') == '1':
            render_engine = 'ogre'
        else:
            render_engine = 'ogre2'

    if render_engine not in ('ogre', 'ogre2'):
        raise RuntimeError(
            f'Unsupported render_engine={render_engine}. Use auto, ogre, or ogre2.'
        )

    # ------------------------------------------------------------------ #
    # MoveIt config
    # ------------------------------------------------------------------ #
    moveit_config = get_moveit_config()

    # ------------------------------------------------------------------ #
    # 1. Gazebo Sim
    #
    # headless=true (默认): 启动 Gazebo server-only 模式 (-s)，不开 GUI 窗口。
    #   - 物理仿真、gz_ros2_control、RGBD 传感器全部正常工作
    #   - 可视化由 RViz2 承担 (软件渲染下比 Gazebo Qt+OGRE GUI 稳得多)
    #   - 解决 WSL2 集显软件渲染下 Gazebo 主窗口
    #     QOpenGLContext::doneCurrent() segfault 问题
    # headless=false: 启动带 GUI 模式，要求宿主机有可用 GPU 直通。
    # ------------------------------------------------------------------ #
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    source_world_file = os.path.join(
        os.environ.get('HOME', '/root'),
        'RobotSim', 'alicia_gz_sim', 'worlds', 'bin_scene.sdf'
    )

    world_file = world_file_for_render_engine(source_world_file, render_engine)

    gz_flags = ['-r', '--render-engine-server', render_engine]
    if headless:
        gz_flags.append('-s')
        if render_engine == 'ogre2':
            gz_flags.append('--headless-rendering')

    gz_args = ' '.join([*gz_flags, world_file])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. robot_state_publisher
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # ------------------------------------------------------------------ #
    # 2.5 Spawn robot in Gazebo (from /robot_description)
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'alicia_d_fixed',
            '-allow_renaming', 'true'
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 3. Controller spawner
    # ------------------------------------------------------------------ #
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'Alicia_controller',
            'Gripper_controller',
            '-c', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # spawn_robot 进程退出后, gz_ros2_control 仍需若干秒在 ign gazebo
    # 进程内初始化 controller_manager 和 hardware interface。
    # 使用单个 spawner 一次性加载并激活所有控制器，避免串联 spawner
    # 在 controller_manager 初始化 race window 中产生半初始化状态。
    # ------------------------------------------------------------------ #
    delay_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=12.0, actions=[controller_spawner]),
            ],
        )
    )

    # ------------------------------------------------------------------ #
    # 4. MoveIt move_group (等所有控制器就绪)
    # ------------------------------------------------------------------ #
    move_group_params = {
        'allow_trajectory_execution': True,
        'capabilities': '',
        'disable_capabilities': '',
        'monitor_dynamics': False,

        # Gazebo on small boards can run much slower than wall clock. The
        # ros2_control trajectory action uses sim time and can still complete,
        # while MoveIt wall-clock duration monitoring may cancel it early.
        'trajectory_execution.allowed_execution_duration_scaling': 20.0,
        'trajectory_execution.allowed_goal_duration_margin': 10.0,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_params,
            {'use_sim_time': True},
        ],
    )

    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # ------------------------------------------------------------------ #
    # 5. ros_gz_bridge — 相机数据桥接
    #
    # ogre1 兼容方案: rgbd_camera 拆为独立 camera + depth_camera 两个传感器后，
    # GZ 端实际发布:
    #   /rgbd_camera/image                  (camera 类型: ignition.msgs.Image)
    #   /rgbd_camera/camera_info            (camera/depth_camera 类型: ignition.msgs.CameraInfo)
    #   /rgbd_camera/depth_image            (depth_camera 类型: ignition.msgs.Image)
    # 用 ros_gz_bridge.yaml 显式声明 GZ topic 与 ROS topic, 避免 node-level
    # remap 对 parameter_bridge 内部 advertise 的 topic 不生效。
    # ------------------------------------------------------------------ #
    bridge_config_file = os.path.join(
        get_package_share_directory('alicia_moveit_config'),
        'config',
        'ros_gz_bridge.yaml',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
        }],
    )

    # ------------------------------------------------------------------ #
    # 6. static_transform_publisher — world -> rgbd_camera_frame
    #    rgbd_camera_frame follows the optical convention:
    #      +Z = depth direction (downward in world), +X = image right, +Y = image down.
    # ------------------------------------------------------------------ #
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_camera_tf',
        output='screen',
        arguments=[
            '--x', '-0.36',
            '--y', '0.18',
            '--z', '0.85',
            '--roll', '3.141592653589793',
            '--pitch', '0.0',
            '--yaw', '-1.5707963267948966',
            '--frame-id', 'world',
            '--child-frame-id', 'rgbd_camera_frame',
        ],
    )

    # ------------------------------------------------------------------ #
    # 9. RViz (可选)
    # ------------------------------------------------------------------ #
    pkg_share = get_package_share_directory('alicia_moveit_config')
    rviz_config_file = os.path.join(pkg_share, 'config', 'demo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    # ------------------------------------------------------------------ #
    # Assemble entities
    #
    # SetParameter 必须放在最前面，作用于其后所有 Node。
    # 关键: gz_ros2_control 的 /joint_states 时间戳走 Gazebo sim_time (从 0 开始),
    # 默认走 wall clock 的 ROS 节点(MoveIt / RViz / RSP / TF) 会认为关节状态
    # "比当前时间老 56 年", 拒绝执行轨迹 ("Failed to validate trajectory:
    # couldn't receive full current joint state within 1s" -> ABORTED)。
    # 解决: 让所有节点订阅 /clock 并使用 sim_time, 与 Gazebo 时基对齐。
    # ------------------------------------------------------------------ #
    entities = [
        SetParameter(name='use_sim_time', value=True),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        delay_controller_spawner,
        delay_move_group,
        ros_gz_bridge,
        camera_tf_publisher,
    ]

    if use_rviz:
        entities.append(rviz_node)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description=(
                'Run Gazebo in server-only mode (no GUI window). '
                'Default true for WSL2 software rendering compatibility. '
                'Set false only when GPU passthrough is available.'
            ),
        ),
        DeclareLaunchArgument(
            'render_engine',
            default_value='auto',
            description=(
                'Gazebo sensor render engine: auto, ogre, or ogre2. '
                'auto uses ogre for software rendering and ogre2 otherwise.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
