"""
Bringup launch file for the ArUco corner pinpointing demo (TIAGo Pro simulation).

Starts Gazebo with TIAGo Pro, spawns an ArUco marker model, and launches the
Agimus MPC controller.

Two modes for the aruco_marker TF:
  use_aruco_detection:=true  (default)
      Live detection from the head camera via aruco_node. The marker pose
      updates continuously — may be noisy depending on lighting and distance.
  use_aruco_detection:=false
      Static TF fixed at the spawn position (roll=π/2, pitch=0, yaw=-π/2 to
      match the OpenCV ArUco frame convention). Stable, no perception noise.

Both modes produce an aruco_marker frame with the same convention:
    X: right of marker face (as seen from robot)
    Y: up on marker face
    Z: toward the robot (approach direction)

Usage:
    ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py
    ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py \\
        use_aruco_detection:=false marker_x:=0.6 marker_z:=1.2
"""

import os
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from agimus_demos_common.launch_utils import get_use_sim_time
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_mpc_debugger = LaunchConfiguration("use_mpc_debugger")
    use_aruco_detection = LaunchConfiguration("use_aruco_detection")

    # ------------------------------------------------------------------ #
    # TIAGo Pro (simulation or hardware) with Linear Feedback Controller
    # ------------------------------------------------------------------ #
    tiago_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("tiago_pro_lfc_bringup"),
                    "launch",
                    "tiago_pro_common.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_gazebo": LaunchConfiguration("use_gazebo"),
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Spawn ArUco marker model in Gazebo
    # Generate the texture first with: scripts/generate_aruco_texture.py
    # ------------------------------------------------------------------ #
    aruco_model_path = os.path.join(
        get_package_share_directory("agimus_demo_04_aruco_corners"),
        "models", "aruco_marker_0", "model.sdf",
    )
    spawn_aruco_marker_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "aruco_marker_0",
            "-file", aruco_model_path,
            "-x", LaunchConfiguration("marker_x"),
            "-y", LaunchConfiguration("marker_y"),
            "-z", LaunchConfiguration("marker_z"),
            "-P", str(-math.pi / 2),  # face pointing toward robot (+X direction)
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # ArUco marker TF — two modes (controlled by use_aruco_detection)
    #
    # Mode A (use_aruco_detection:=false): static TF base_footprint → aruco_marker
    #   Pose is fixed at the spawn coordinates. Stable, no perception noise.
    #
    # Mode B (use_aruco_detection:=true): live detection from head camera.
    #   aruco_node detects DICT_4X4_50 markers and publishes /aruco_markers.
    #   aruco_corner_publisher subscribes and re-broadcasts as TF.
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
            # RPY matching the OpenCV ArUco frame convention:
            # X=right, Y=up, Z=toward robot.
            "--roll", str(math.pi / 2),
            "--pitch", "0.0",
            "--yaw", str(-math.pi / 2),
        ],
        condition=UnlessCondition(use_aruco_detection),
        output="screen",
    )

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

    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": "robot_srdf_description",
                "string_value": ParameterValue(
                    Command(
                        [
                            PathJoinSubstitution([FindExecutable(name="xacro")]),
                            " ",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("agimus_demos_common"),
                                    "config",
                                    "tiago_pro",
                                    "tiago_pro_dummy.srdf.xacro",
                                ]
                            ),
                        ]
                    ),
                    value_type=str,
                ),
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
        node_kwargs=dict(condition=IfCondition(use_mpc_debugger)),
    )

    # ------------------------------------------------------------------ #
    # Launch sequence: start aruco detection immediately, then wait for
    # robot joints before starting the MPC controller and corner publisher.
    # ------------------------------------------------------------------ #
    return [
        tiago_robot_launch,
        spawn_aruco_marker_node,
        static_aruco_tf_node,
        aruco_node,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    robot_srdf_publisher_node,
                    agimus_controller_node,
                    aruco_corner_publisher_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    models_dir = os.path.join(
        get_package_share_directory("agimus_demo_04_aruco_corners"), "models"
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_dir),
            DeclareLaunchArgument(
                "use_mpc_debugger",
                default_value="false",
                choices=["true", "false"],
                description="Launch the mpc_debugger_node for RViz prediction visualisation.",
            ),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="true",
                choices=["true", "false"],
                description="Launch with Gazebo simulation.",
            ),
            DeclareLaunchArgument(
                "use_aruco_detection",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "true: live ArUco detection from head camera (may be noisy). "
                    "false: static TF fixed at spawn coordinates (stable)."
                ),
            ),
            # ArUco marker position in the world frame.
            # Default: in front of TIAGo Pro at arm height.
            # Orientation is hardcoded: model spawned with pitch=-π/2 (face toward robot),
            # static TF with roll=π/2, yaw=-π/2 (OpenCV ArUco convention).
            DeclareLaunchArgument(
                "marker_x", default_value="0.6",
                description="X position of the ArUco marker in world frame (m).",
            ),
            DeclareLaunchArgument(
                "marker_y", default_value="0.0",
                description="Y position of the ArUco marker in world frame (m).",
            ),
            DeclareLaunchArgument(
                "marker_z", default_value="1.2",
                description="Z position of the ArUco marker in world frame (m).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
