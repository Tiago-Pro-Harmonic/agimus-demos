"""
Bringup launch file for the TIAGo Pro pick-and-place demo (simulation).

Starts Gazebo with TIAGo Pro and the Agimus MPC controller configured for
whole-body control with a planar mobile base (x, y, yaw) + right arm (7 DoF).

Usage:
    ros2 launch agimus_demo_06_tiago_pro_pick_and_place bringup_simu.launch.py
    ros2 launch agimus_demo_06_tiago_pro_pick_and_place bringup_simu.launch.py \\
        use_mpc_debugger:=true
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

PKG = "agimus_demo_06_tiago_pro_pick_and_place"

# ── Load scene geometry from shared config (single source of truth) ────────
_cfg_path = os.path.join(
    os.path.dirname(__file__), "..", "config", "hpp_orchestrator_params.yaml"
)
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)

_scene          = _cfg["scene"]
TABLE_OFFSET_X  = _scene["table_offset_x"]
TABLE_OFFSET_Z  = _scene["table_offset_z"]
BOX_Y_INIT      = _scene["box_y_init"]
_TABLE_Z        = 0.730 + TABLE_OFFSET_Z          # table surface height
_BOX_HALF       = 0.025
_BOX_Z          = _TABLE_Z + _BOX_HALF            # box centre z
_TABLE_NEAR_EDGE = 0.8 + TABLE_OFFSET_X - 0.3
_BOX_X          = _TABLE_NEAR_EDGE + _BOX_HALF + 0.02


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
    # Environment description (empty for now — table/box added later)
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
    # MPC controller (planar base + right arm)
    # ------------------------------------------------------------------ #
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [FindPackageShare(PKG), "config", "agimus_controller_params.yaml"]
            ),
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Base command bridge: MPC base velocities → /cmd_vel
    # ------------------------------------------------------------------ #
    base_cmd_bridge_node = Node(
        package=PKG,
        executable="base_cmd_bridge",
        name="base_cmd_bridge",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Scene: spawn table and box — positions derived from hpp_orchestrator_params.yaml
    # ------------------------------------------------------------------ #
    table_urdf = PathJoinSubstitution(
        [FindPackageShare(PKG), "urdf", "table.urdf"]
    )
    spawn_table_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "table",
            "-file", table_urdf,
            "-x", str(TABLE_OFFSET_X),
            "-y", "0.0",
            "-z", str(TABLE_OFFSET_Z),
        ],
        output="screen",
    )

    box_urdf = PathJoinSubstitution(
        [FindPackageShare(PKG), "urdf", "box.urdf"]
    )
    spawn_box_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "box",
            "-file", box_urdf,
            "-x", str(_BOX_X),
            "-y", str(BOX_Y_INIT),
            "-z", str(_BOX_Z),
        ],
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

    # ------------------------------------------------------------------ #
    # HPP trajectory bridge (optional)
    #
    # Opens an xterm that sources both configs in order:
    #   1. ros2_config.sh  → ROS2 libs in LD_LIBRARY_PATH + rclpy available
    #   2. hpp_config.sh   → HPP Python + libs prepended (takes priority)
    # The bridge script then adds ros2_ws install paths to sys.path itself
    # to import agimus_msgs.
    #
    # Launched after the agimus_controller is ready (OnProcessExit of
    # wait_for_non_zero_joints_node) so /mpc_input has a subscriber.
    # ------------------------------------------------------------------ #
    use_hpp_bridge = LaunchConfiguration("use_hpp_bridge")
    hpp_bridge_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-T", "HPP orchestrator",
            "-e",
            "bash -c '"
            "source /home/user/devel/ros2_config.sh && "
            "source /home/user/devel/ros2_ws/install/setup.bash && "
            "source /home/user/devel/hpp_config.sh && "
            "python3 /home/user/devel/ros2_ws/install/agimus_demo_06_tiago_pro_pick_and_place"
            "/share/agimus_demo_06_tiago_pro_pick_and_place/hpp/orchestrator_node.py"
            "'",
        ],
        output="screen",
        condition=IfCondition(use_hpp_bridge),
    )

    return [
        tiago_robot_launch,
        spawn_table_node,
        spawn_box_node,
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
            DeclareLaunchArgument(
                "use_hpp_bridge",
                default_value="false",
                choices=["true", "false"],
                description="Launch the HPP orchestrator shell in an xterm terminal.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
