import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_TARGET_SEQUENCE = ["red_cylinder", "yellow_box", "brown_cube"]

TOP_DOWN_GRASP_HEIGHT_OFFSETS = {
    "red_cylinder": -0.075,
    "yellow_box": -0.040,
    "brown_cube": -0.040,
}

DEFAULT_PLACE_SLOTS_XYZ = {
    "red_cylinder": [-0.36, -0.145, 0.28],
    "yellow_box": [-0.31, -0.145, 0.28],
    "brown_cube": [-0.26, -0.145, 0.28],
}


def parse_list(text, fallback):
    try:
        value = ast.literal_eval(text)
        if not isinstance(value, list):
            raise ValueError
        return value
    except Exception:
        return list(fallback)


def parse_float_list(text, fallback):
    try:
        return [float(v) for v in ast.literal_eval(text)]
    except Exception:
        return list(fallback)


def default_grasp_height_offset(target_name, grasp_mode, grasp_style):
    if grasp_mode == "physical" and grasp_style == "top_down":
        return TOP_DOWN_GRASP_HEIGHT_OFFSETS.get(target_name, -0.045)
    if grasp_mode == "physical" and grasp_style == "strict_top_down":
        return -0.075
    if grasp_mode == "physical" and grasp_style == "side":
        return -0.045
    return 0.035


def build_default_place_slots(target_sequence, fallback_place_xyz):
    values = []
    for target_name in target_sequence:
        values.extend(DEFAULT_PLACE_SLOTS_XYZ.get(target_name, fallback_place_xyz))
    return values


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    grasp_mode = LaunchConfiguration("grasp_mode").perform(context).strip().lower()
    if grasp_mode not in ("virtual", "physical"):
        grasp_mode = "virtual"

    grasp_style = LaunchConfiguration("grasp_style").perform(context).strip().lower()
    if grasp_style not in ("auto", "side", "top_down", "strict_top_down"):
        grasp_style = "auto"
    if grasp_style == "auto":
        grasp_style = "top_down" if grasp_mode == "physical" else "side"

    legacy_virtual_grasp = (
        LaunchConfiguration("virtual_grasp").perform(context).lower() == "true"
    )
    virtual_grasp = grasp_mode == "virtual" and legacy_virtual_grasp

    target_sequence = [
        str(name)
        for name in parse_list(
            LaunchConfiguration("target_sequence").perform(context),
            DEFAULT_TARGET_SEQUENCE,
        )
    ]

    place_xyz = parse_float_list(
        LaunchConfiguration("place_xyz").perform(context),
        [-0.36, -0.145, 0.28],
    )
    if len(place_xyz) != 3:
        place_xyz = [-0.36, -0.145, 0.28]

    carry_lift_z = float(LaunchConfiguration("carry_lift_z").perform(context))
    pregrasp_height_text = (
        LaunchConfiguration("pregrasp_height").perform(context).strip().lower()
    )
    grasp_height_text = (
        LaunchConfiguration("grasp_height_offset").perform(context).strip().lower()
    )
    grasp_height_offsets_text = (
        LaunchConfiguration("grasp_height_offsets").perform(context).strip()
    )
    grasp_offset_text = LaunchConfiguration("grasp_position_offset_xyz").perform(context)
    grasp_reference_offset_text = LaunchConfiguration(
        "grasp_reference_offset_xyz"
    ).perform(context)
    grasp_rpy_text = LaunchConfiguration("grasp_rpy_deg").perform(context)
    place_slots_text = LaunchConfiguration("place_slots_xyz").perform(context).strip()
    max_declutter_text = (
        LaunchConfiguration("max_declutter_per_target").perform(context).strip().lower()
    )
    goal_pose_joint_fallback_text = (
        LaunchConfiguration("goal_pose_joint_fallback").perform(context).strip().lower()
    )
    goal_pose_joint_fallback_min_z_text = (
        LaunchConfiguration("goal_pose_joint_fallback_min_z")
        .perform(context)
        .strip()
        .lower()
    )
    gripper_close_text = (
        LaunchConfiguration("gripper_close_position").perform(context).strip().lower()
    )
    gripper_approach_text = (
        LaunchConfiguration("gripper_approach_position").perform(context).strip().lower()
    )

    top_down_physical = grasp_mode == "physical" and grasp_style == "top_down"
    strict_top_down_physical = (
        grasp_mode == "physical" and grasp_style == "strict_top_down"
    )
    side_physical = grasp_mode == "physical" and grasp_style == "side"

    if pregrasp_height_text == "auto":
        if top_down_physical or strict_top_down_physical:
            pregrasp_height = 0.20 if top_down_physical else 0.22
        elif side_physical:
            pregrasp_height = 0.20
        else:
            pregrasp_height = 0.10
    else:
        pregrasp_height = float(pregrasp_height_text)

    if gripper_close_text == "auto":
        if top_down_physical:
            gripper_close_position = 0.040
        elif strict_top_down_physical:
            gripper_close_position = 0.020
        elif side_physical:
            gripper_close_position = 0.030
        else:
            gripper_close_position = 0.05
    else:
        gripper_close_position = float(gripper_close_text)

    if gripper_approach_text == "auto":
        # Pre-descent partial-open. ~8.3cm gap (>7cm cylinder dia) but
        # 2cm less outer pad span than the full-open posture, so finger pads
        # are less likely to clip a neighbor on the way down.
        gripper_approach_position = 0.010
    else:
        gripper_approach_position = float(gripper_approach_text)

    if grasp_height_text == "auto":
        first_target = target_sequence[0] if target_sequence else "red_cylinder"
        grasp_height_offset = default_grasp_height_offset(
            first_target,
            grasp_mode,
            grasp_style,
        )
        grasp_height_offset_is_auto = True
    else:
        grasp_height_offset = float(grasp_height_text)
        grasp_height_offset_is_auto = False

    if grasp_height_offsets_text.lower() == "auto":
        if grasp_height_offset_is_auto:
            grasp_height_offsets = [
                default_grasp_height_offset(name, grasp_mode, grasp_style)
                for name in target_sequence
            ]
        else:
            grasp_height_offsets = [grasp_height_offset] * len(target_sequence)
    else:
        grasp_height_offsets = parse_float_list(grasp_height_offsets_text, [])
        if len(grasp_height_offsets) == 1 and len(target_sequence) > 1:
            grasp_height_offsets = grasp_height_offsets * len(target_sequence)
        if len(grasp_height_offsets) != len(target_sequence):
            grasp_height_offsets = [
                default_grasp_height_offset(name, grasp_mode, grasp_style)
                for name in target_sequence
            ]

    if grasp_offset_text.strip().lower() == "auto":
        if side_physical:
            grasp_position_offset_xyz = [-0.030, 0.0, 0.0]
        else:
            grasp_position_offset_xyz = [0.0, 0.0, 0.0]
    else:
        grasp_position_offset_xyz = parse_float_list(grasp_offset_text, [0.0, 0.0, 0.0])
        if len(grasp_position_offset_xyz) != 3:
            grasp_position_offset_xyz = [0.0, 0.0, 0.0]

    if grasp_rpy_text.strip().lower() == "auto":
        if strict_top_down_physical:
            grasp_rpy_deg = [180.0, 0.0, 90.0]
        else:
            grasp_rpy_deg = [180.0, 40.0, 0.0]
    else:
        grasp_rpy_deg = parse_float_list(grasp_rpy_text, [180.0, 90.0, 0.0])
        if len(grasp_rpy_deg) != 3:
            grasp_rpy_deg = [180.0, 40.0, 0.0]

    if grasp_reference_offset_text.strip().lower() == "auto":
        grasp_reference_offset_xyz = [0.0, 0.0, 0.0]
    else:
        grasp_reference_offset_xyz = parse_float_list(
            grasp_reference_offset_text,
            [0.0, 0.0, 0.0],
        )
        if len(grasp_reference_offset_xyz) != 3:
            grasp_reference_offset_xyz = [0.0, 0.0, 0.0]

    if place_slots_text.lower() == "auto":
        place_slots_xyz = build_default_place_slots(target_sequence, place_xyz)
    else:
        place_slots_xyz = parse_float_list(place_slots_text, [])
        if len(place_slots_xyz) % 3 != 0:
            place_slots_xyz = build_default_place_slots(target_sequence, place_xyz)

    if max_declutter_text == "auto":
        max_declutter_per_target = 0 if grasp_mode == "physical" else 1
    else:
        max_declutter_per_target = int(max_declutter_text)

    if goal_pose_joint_fallback_text == "auto":
        goal_pose_joint_fallback = True
    else:
        goal_pose_joint_fallback = goal_pose_joint_fallback_text == "true"

    if goal_pose_joint_fallback_min_z_text == "auto":
        goal_pose_joint_fallback_min_z = -1.0
    else:
        goal_pose_joint_fallback_min_z = float(goal_pose_joint_fallback_min_z_text)

    nodes = [
        Node(
            package="perception_mvp",
            executable="color_perception_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "pregrasp_height": pregrasp_height,
                }
            ],
        ),
        Node(
            package="perception_mvp",
            executable="move_to_pregrasp_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "goal_pose_cartesian": grasp_mode == "physical",
                    "goal_pose_joint_fallback": goal_pose_joint_fallback,
                    "goal_pose_joint_fallback_min_z": goal_pose_joint_fallback_min_z,
                    "gripper_motion_time_sec": 5.0 if grasp_mode == "physical" else 3.0,
                    "grasp_roll_deg": grasp_rpy_deg[0],
                    "grasp_pitch_deg": grasp_rpy_deg[1],
                    "grasp_yaw_deg": grasp_rpy_deg[2],
                    "grasp_reference_offset_xyz": grasp_reference_offset_xyz,
                }
            ],
        ),
        Node(
            package="perception_mvp",
            executable="task_state_machine_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "target_sequence": target_sequence,
                    "place_xyz": place_xyz,
                    "place_slots_xyz": place_slots_xyz,
                    "declutter_temp_xyz": place_xyz,
                    "carry_lift_z": carry_lift_z,
                    "grasp_height_offset": grasp_height_offset,
                    "grasp_height_offsets": grasp_height_offsets,
                    "grasp_position_offset_xyz": grasp_position_offset_xyz,
                    "grasp_reference_offset_xyz": grasp_reference_offset_xyz,
                    "grasp_rpy_deg": grasp_rpy_deg,
                    "place_rpy_deg": grasp_rpy_deg,
                    "declutter_temp_rpy_deg": grasp_rpy_deg,
                    "gripper_close_position": gripper_close_position,
                    "gripper_approach_position": gripper_approach_position,
                    "max_declutter_per_target": max_declutter_per_target,
                }
            ],
        ),
    ]

    if virtual_grasp:
        nodes.insert(
            2,
            Node(
                package="perception_mvp",
                executable="virtual_grasp_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "grasp_mode",
            default_value="virtual",
            description="virtual uses set_pose attachment; physical uses Gazebo collision/friction only",
        ),
        DeclareLaunchArgument(
            "grasp_style",
            default_value="auto",
            description="auto: top_down in physical mode, side in virtual mode; options: side, top_down, strict_top_down",
        ),
        DeclareLaunchArgument("virtual_grasp", default_value="true"),
        DeclareLaunchArgument(
            "target_sequence",
            default_value="['red_cylinder','yellow_box','brown_cube']",
            description="Python-list string, for example ['red_cylinder','yellow_box']",
        ),
        DeclareLaunchArgument(
            "place_xyz",
            default_value="[-0.36, -0.145, 0.28]",
            description="Fallback Bin B release hover [x, y, z] in world frame",
        ),
        DeclareLaunchArgument(
            "place_slots_xyz",
            default_value="auto",
            description="Flat xyz list for per-target Bin B slots; auto follows target_sequence",
        ),
        DeclareLaunchArgument(
            "carry_lift_z",
            default_value="0.32",
            description="Absolute world z for the post-grasp carry lift",
        ),
        DeclareLaunchArgument(
            "max_declutter_per_target",
            default_value="auto",
            description="auto: 0 in physical mode, 1 in virtual mode",
        ),
        DeclareLaunchArgument(
            "pregrasp_height",
            default_value="auto",
            description="auto: 0.10 in virtual, 0.20 in physical top_down/side, 0.22 in strict_top_down",
        ),
        DeclareLaunchArgument(
            "grasp_height_offset",
            default_value="auto",
            description="Legacy single target z offset; ignored per target when grasp_height_offsets is auto",
        ),
        DeclareLaunchArgument(
            "grasp_height_offsets",
            default_value="auto",
            description="Per-target z offsets aligned with target_sequence; auto uses object geometry defaults",
        ),
        DeclareLaunchArgument(
            "grasp_position_offset_xyz",
            default_value="auto",
            description="auto: [-0.030,0,0] in physical side, [0,0,0] otherwise",
        ),
        DeclareLaunchArgument(
            "grasp_reference_offset_xyz",
            default_value="auto",
            description="auto: [0,0,0]; override only when tuning finger-pad reference geometry",
        ),
        DeclareLaunchArgument(
            "grasp_rpy_deg",
            default_value="auto",
            description="auto: [180,90,0] except strict_top_down uses [180,0,90]",
        ),
        DeclareLaunchArgument(
            "goal_pose_joint_fallback",
            default_value="auto",
            description="auto: true; top_down uses goal_pose_joint_fallback_min_z to keep low grasp descent Cartesian-only",
        ),
        DeclareLaunchArgument(
            "goal_pose_joint_fallback_min_z",
            default_value="auto",
            description="auto: -1.0; joint fallback is allowed if Cartesian planning cannot cover the pose",
        ),
        DeclareLaunchArgument(
            "gripper_close_position",
            default_value="auto",
            description="auto: 0.05 in virtual, 0.030 in physical side, 0.040 in physical top_down, 0.020 in strict_top_down",
        ),
        DeclareLaunchArgument(
            "gripper_approach_position",
            default_value="auto",
            description="auto: 0.010; partial-open opening before descent so finger pads stick out less and avoid clipping neighbor objects",
        ),
        OpaqueFunction(function=launch_setup),
    ])
