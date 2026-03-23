"""
Bringup launch file for the TIAGo Pro planar-base demo (simulation).

Starts Gazebo with TIAGo Pro and the Agimus MPC controller configured for
whole-body control with a planar mobile base (x, y, yaw) + right arm (7 DoF).

Usage:
    ros2 launch agimus_demo_tiago_pro_planar_base bringup_simu.launch.py
    ros2 launch agimus_demo_tiago_pro_planar_base bringup_simu.launch.py \\
        use_mpc_debugger:=true
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
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

    # ------------------------------------------------------------------ #
    # TIAGo Pro (simulation) with Linear Feedback Controller
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
            "gzclient": LaunchConfiguration("gzclient"),
            "lfc_yaml": "config/free_flyer/linear_feedback_controller_params.yaml",
            "jse_yaml": "config/free_flyer/joint_state_estimator_params.yaml",
            "pc_yaml": "config/free_flyer/dummy_controllers.yaml",
        }.items(),
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
                    FindPackageShare("agimus_demo_tiago_pro_planar_base"),
                    "config",
                    "agimus_controller_params.yaml",
                ]
            ),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Fixed-goal publisher
    # ------------------------------------------------------------------ #
    fixed_goal_publisher_node = Node(
        package="agimus_demo_tiago_pro_planar_base",
        executable="fixed_goal_publisher",
        name="fixed_goal_publisher",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demo_tiago_pro_planar_base"),
                    "config",
                    "trajectory_weights_params.yaml",
                ]
            ),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Base command bridge: forwards MPC base velocities to mobile base controller
    # ------------------------------------------------------------------ #
    base_cmd_bridge_node = Node(
        package="agimus_demo_tiago_pro_planar_base",
        executable="base_cmd_bridge",
        name="base_cmd_bridge",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Optional MPC debugger (visualises predicted trajectory in RViz)
    # ------------------------------------------------------------------ #
    mpc_debugger = mpc_debugger_node(
        "gripper_right_fingertip_left_link",
        parent_frame="odom",
        cost_plot=True,
        node_kwargs=dict(condition=IfCondition(use_mpc_debugger)),
    )

    return [
        tiago_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    robot_srdf_publisher_node,
                    agimus_controller_node,
                    fixed_goal_publisher_node,
                    base_cmd_bridge_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
                "gzclient",
                default_value="True",
                choices=["True", "False"],
                description="Launch Gazebo GUI client.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
