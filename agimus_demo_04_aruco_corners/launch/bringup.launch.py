"""
Bringup launch file for the ArUco corner pinpointing demo (TIAGo Pro real robot).

Connects to the real TIAGo Pro with the Linear Feedback Controller, launches the
Agimus MPC controller and the ArUco corner publisher node.

Two modes for the aruco_marker TF:
  use_aruco_detection:=true  (default)
      Live detection from the head camera via aruco_node. The marker pose
      updates continuously — may be noisy depending on lighting and distance.
  use_aruco_detection:=false
      Static TF fixed at the given marker position (base_footprint → aruco_marker).
      Stable, no perception noise.

Usage:
    ros2 launch agimus_demo_04_aruco_corners bringup.launch.py
    ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \\
        use_aruco_detection:=false marker_x:=0.6 marker_z:=1.2
"""

import math

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_mpc_debugger = LaunchConfiguration("use_mpc_debugger")
    use_aruco_detection = LaunchConfiguration("use_aruco_detection")

    # ------------------------------------------------------------------ #
    # TIAGo Pro with Linear Feedback Controller
    # ------------------------------------------------------------------ #
    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={"tuck_arm": "False"},
    )

    # ------------------------------------------------------------------ #
    # Static TF: ArUco marker pose (base_footprint → aruco_marker)
    # Used when use_aruco_detection:=false.
    # ------------------------------------------------------------------ #
    static_aruco_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "base_footprint",
            "--child-frame-id", "aruco_marker",
            "--x", LaunchConfiguration("marker_x"),
            "--y", LaunchConfiguration("marker_y"),
            "--z", LaunchConfiguration("marker_z"),
            "--roll", LaunchConfiguration("marker_roll"),
            "--pitch", LaunchConfiguration("marker_pitch"),
            "--yaw", LaunchConfiguration("marker_yaw"),
        ],
        condition=UnlessCondition(use_aruco_detection),
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # ArUco detection node (live detection from head camera)
    # ------------------------------------------------------------------ #
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        parameters=[
            {
                "marker_size": 0.176,
                "aruco_dictionary_id": "DICT_4X4_50",
                "image_topic": "/head_front_camera/image",
                "camera_info_topic": "/head_front_camera/camera_info",
            }
        ],
        condition=IfCondition(use_aruco_detection),
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Synchronisation: wait until the robot has non-zero joint positions
    # ------------------------------------------------------------------ #
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Environment description (empty — no collision objects in this demo)
    # ------------------------------------------------------------------ #
    environment_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="environment_publisher",
        parameters=[
            {
                "topic_name": "environment_description",
                "string_value": "<robot name='empty'><link name='env'/></robot>",
            }
        ],
    )

    # ------------------------------------------------------------------ #
    # MPC controller
    # ------------------------------------------------------------------ #
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demo_04_aruco_corners"),
                    "config",
                    "agimus_controller_params.yaml",
                ]
            ),
        ],
        remappings=[
            ("robot_description_semantic", "robot_srdf_description"),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # ArUco corner publisher
    # ------------------------------------------------------------------ #
    aruco_corner_publisher_node = Node(
        package="agimus_demo_04_aruco_corners",
        executable="aruco_corner_publisher",
        name="aruco_corner_publisher",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demo_04_aruco_corners"),
                    "config",
                    "trajectory_weights_params.yaml",
                ]
            ),
            {"use_aruco_detection": use_aruco_detection},
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Optional MPC debugger (visualises predicted trajectory in RViz)
    # ------------------------------------------------------------------ #
    mpc_debugger = mpc_debugger_node(
        "gripper_right_fingertip_right_link",
        parent_frame="base_footprint",
        cost_plot=True,
        node_kwargs=dict(
            remappings=[
                ("robot_description_semantic", "robot_srdf_description"),
            ],
            condition=IfCondition(use_mpc_debugger),
        ),
    )

    # ------------------------------------------------------------------ #
    # Launch sequence: start aruco detection immediately, then wait for
    # robot joints before starting the MPC controller and corner publisher.
    # ------------------------------------------------------------------ #
    return [
        tiago_robot_launch,
        static_aruco_tf_node,
        aruco_node,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    aruco_corner_publisher_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args()
        + [
            DeclareLaunchArgument(
                "use_mpc_debugger",
                default_value="false",
                choices=["true", "false"],
                description="Launch the mpc_debugger_node for RViz prediction visualisation.",
            ),
            DeclareLaunchArgument(
                "use_aruco_detection",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "true: live ArUco detection from head camera (default for real robot). "
                    "false: static TF fixed at given marker coordinates."
                ),
            ),
            DeclareLaunchArgument(
                "marker_x", default_value="0.6",
                description="X position of the ArUco marker in base_footprint frame (m).",
            ),
            DeclareLaunchArgument(
                "marker_y", default_value="0.0",
                description="Y position of the ArUco marker in base_footprint frame (m).",
            ),
            DeclareLaunchArgument(
                "marker_z", default_value="1.2",
                description="Z position of the ArUco marker in base_footprint frame (m).",
            ),
            DeclareLaunchArgument(
                "marker_roll", default_value=str(math.pi / 2),
                description="Roll of the ArUco marker in base_footprint frame (rad).",
            ),
            DeclareLaunchArgument(
                "marker_pitch", default_value="0.0",
                description="Pitch of the ArUco marker in base_footprint frame (rad).",
            ),
            DeclareLaunchArgument(
                "marker_yaw", default_value=str(-math.pi / 2),
                description="Yaw of the ArUco marker in base_footprint frame (rad).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
