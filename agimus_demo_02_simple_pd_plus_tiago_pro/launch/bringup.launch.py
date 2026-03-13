from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import get_use_sim_time


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
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

    pd_plus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_02_simple_pd_plus_tiago_pro"),
            "config",
            "pd_plus_controller_params.yaml",
        ]
    )

    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[get_use_sim_time(), pd_plus_controller_params],
        output="screen",
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    return [
        tiago_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[pd_plus_controller_node],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="true",
                choices=["true", "false"],
                description="Use Gazebo simulation. If false, connects to real robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
