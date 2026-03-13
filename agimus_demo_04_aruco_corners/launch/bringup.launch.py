"""
Bringup launch file for the ArUco corner pinpointing demo.

Starts the Franka robot (simulation or hardware), the Agimus MPC controller, and
the ArUco corner publisher node.  The ArUco marker pose is fixed via a static TF
transform (fer_link0 → aruco_marker), configurable through launch arguments.

Usage (simulation):
    ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \\
        use_gazebo:=true use_rviz:=true \\
        marker_x:=0.5 marker_y:=0.0 marker_z:=0.3 marker_pitch:=3.14159

The default marker pose places the marker roughly in front of the Franka arm,
facing the robot (Z axis pointing toward it).
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
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
    # Franka robot (simulation or hardware) with Linear Feedback Controller
    # ------------------------------------------------------------------ #
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")

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
    # Static TF: simulated ArUco marker pose (fer_link0 → aruco_marker)
    # Only used in simulation; on the real robot the TF comes from ArUco detection.
    # ------------------------------------------------------------------ #
    aruco_marker_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "fer_link0",
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
    # ArUco detection node (real robot only)
    # Detects markers from the wrist-mounted RealSense D435.
    # Publishes /aruco_markers with poses in camera_color_optical_frame.
    # aruco_corner_publisher subscribes and re-broadcasts as TF.
    # ------------------------------------------------------------------ #
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        parameters=[
            {
                "marker_size": 0.176,
                "aruco_dictionary_id": "DICT_4X4_50",
                "image_topic": "/camera/camera/color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
            }
        ],
        condition=IfCondition(use_aruco_detection),
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Environment description (empty URDF — no collision objects in this demo)
    # The MPC controller requires this topic to initialize the robot model.
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
    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_aruco_corners"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml],
        remappings=[("robot_description", "robot_description_with_collision")],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # ArUco corner publisher
    # ------------------------------------------------------------------ #
    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_aruco_corners"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    )
    aruco_corner_publisher_node = Node(
        package="agimus_demo_04_aruco_corners",
        executable="aruco_corner_publisher",
        name="aruco_corner_publisher",
        parameters=[
            get_use_sim_time(),
            trajectory_weights_yaml,
            {"use_aruco_detection": use_aruco_detection},
        ],
        remappings=[("robot_description", "robot_description_with_collision")],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Optional MPC debugger (visualises predicted trajectory in RViz)
    # ------------------------------------------------------------------ #
    mpc_debugger = mpc_debugger_node(
        "fer_hand_tcp",
        parent_frame="fer_link0",
        cost_plot=True,
        node_kwargs=dict(
            remappings=[("robot_description", "robot_description_with_collision")],
            condition=IfCondition(use_mpc_debugger),
        ),
    )

    # ------------------------------------------------------------------ #
    # Launch sequence: start controller and publisher after robot is ready
    # ------------------------------------------------------------------ #
    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        aruco_marker_tf_node,
        aruco_node,
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
    import math

    declared_arguments = [
        DeclareLaunchArgument(
            "use_mpc_debugger",
            default_value="false",
            description="Launch the mpc_debugger_node for RViz prediction visualisation.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_aruco_detection",
            default_value="false",
            description=(
                "Use live ArUco detection from the wrist D435 camera (real robot). "
                "When false a static TF is used instead (simulation)."
            ),
            choices=["true", "false"],
        ),
        # Marker position in the fer_link0 frame
        DeclareLaunchArgument(
            "marker_x",
            default_value="0.5",
            description="X position of the ArUco marker in fer_link0 frame (m).",
        ),
        DeclareLaunchArgument(
            "marker_y",
            default_value="0.0",
            description="Y position of the ArUco marker in fer_link0 frame (m).",
        ),
        DeclareLaunchArgument(
            "marker_z",
            default_value="0.3",
            description="Z position of the ArUco marker in fer_link0 frame (m).",
        ),
        # Marker orientation: default pitch=pi rotates the marker so its Z axis
        # points back toward the robot (i.e. the marker is facing the robot).
        DeclareLaunchArgument(
            "marker_roll",
            default_value="0.0",
            description="Roll of the ArUco marker in fer_link0 frame (rad).",
        ),
        DeclareLaunchArgument(
            "marker_pitch",
            default_value=str(math.pi),
            description="Pitch of the ArUco marker in fer_link0 frame (rad). "
                        "Default pi: marker Z axis pointing toward the robot.",
        ),
        DeclareLaunchArgument(
            "marker_yaw",
            default_value="0.0",
            description="Yaw of the ArUco marker in fer_link0 frame (rad).",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
