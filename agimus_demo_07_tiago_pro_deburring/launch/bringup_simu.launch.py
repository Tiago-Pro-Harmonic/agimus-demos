"""
Bringup launch file for the TIAGo Pro deburring demo (simulation).

Usage:
    ros2 launch agimus_demo_07_tiago_pro_deburring bringup_simu.launch.py
    ros2 launch agimus_demo_07_tiago_pro_deburring bringup_simu.launch.py use_hpp_bridge:=true
"""

import os
import yaml

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import get_use_sim_time
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node

PKG = "agimus_demo_07_tiago_pro_deburring"

_cfg_path = os.path.join(
    os.path.dirname(__file__), "..", "config", "hpp_orchestrator_params.yaml"
)
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)

_s       = _cfg["scene"]
TABLE_X  = _s["table_x"]
TABLE_Y  = _s["table_y"]
TABLE_Z  = _s["table_z"]
PYLONE_X = TABLE_X
PYLONE_Y = TABLE_Y
PYLONE_Z = _s["pylone_z_offset"]


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    tiago_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tiago_pro_lfc_bringup"),
                "launch",
                "tiago_pro_common.launch.py",
            ])
        ),
        launch_arguments={
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "gzclient":   LaunchConfiguration("gzclient"),
            "lfc_yaml":   "config/free_flyer/linear_feedback_controller_params.yaml",
            "jse_yaml":   "config/free_flyer/joint_state_estimator_params.yaml",
            "pc_yaml":    "config/free_flyer/dummy_controllers.yaml",
        }.items(),
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    environment_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="environment_publisher",
        parameters=[{
            "topic_name":   "environment_description",
            "string_value": "<robot name='empty'><link name='env'/></robot>",
        }],
    )

    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[{
            "topic_name": "robot_srdf_description",
            "string_value": ParameterValue(
                Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution([
                        FindPackageShare("agimus_demos_common"),
                        "config", "tiago_pro", "tiago_pro_dummy.srdf.xacro",
                    ]),
                ]),
                value_type=str,
            ),
        }],
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution([FindPackageShare(PKG), "config", "agimus_controller_params.yaml"]),
        ],
        output="screen",
    )

    base_cmd_bridge_node = Node(
        package=PKG,
        executable="base_cmd_bridge",
        name="base_cmd_bridge",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    spawn_table_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "table",
            "-file", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "table.urdf"]),
            "-x", str(TABLE_X), "-y", str(TABLE_Y), "-z", str(TABLE_Z),
        ],
        output="screen",
    )

    spawn_pylone_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "pylone",
            "-file", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "pylone.urdf"]),
            "-x", str(PYLONE_X), "-y", str(PYLONE_Y), "-z", str(PYLONE_Z),
        ],
        output="screen",
    )

    mpc_debugger = mpc_debugger_node(
        "gripper_right_fingertip_left_link",
        parent_frame="odom",
        cost_plot=True,
        node_kwargs=dict(condition=IfCondition(LaunchConfiguration("use_mpc_debugger"))),
    )

    hpp_bridge_node = ExecuteProcess(
        cmd=[
            "xterm", "-hold", "-T", "HPP orchestrator", "-e",
            "bash -c '"
            "source /home/user/devel/ros2_config.sh && "
            "source /home/user/devel/ros2_ws/install/setup.bash && "
            "source /home/user/devel/hpp_config.sh && "
            "python3 /home/user/devel/ros2_ws/install/agimus_demo_07_tiago_pro_deburring"
            "/share/agimus_demo_07_tiago_pro_deburring/hpp/orchestrator_node.py"
            "'",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_hpp_bridge")),
    )

    return [
        tiago_robot_launch,
        spawn_table_node,
        spawn_pylone_node,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    robot_srdf_publisher_node,
                    agimus_controller_node,
                    base_cmd_bridge_node,
                    hpp_bridge_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mpc_debugger", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("use_gazebo", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("gzclient", default_value="True",
                              choices=["True", "False"]),
        DeclareLaunchArgument("use_hpp_bridge", default_value="false",
                              choices=["true", "false"],
                              description="Launch the HPP orchestrator shell in xterm."),
        OpaqueFunction(function=launch_setup),
    ])
