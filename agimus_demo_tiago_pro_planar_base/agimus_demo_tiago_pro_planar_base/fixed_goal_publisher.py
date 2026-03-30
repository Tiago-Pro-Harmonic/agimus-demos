"""
FixedGoalPublisher — reference publisher for the TIAGo Pro planar-base demo.

Drives the right-arm end-effector to a single fixed Cartesian target while the
planar base is free to move. The publisher continuously sends a constant
WeightedTrajectoryPoint; the MPC optimises both the arm torques and the base
velocity commands to minimise the end-effector position error.

NOTE: This node requires the following infrastructure changes (not yet merged):
  - agimus_controller: planar_base support (JointModelPlanar + mixed actuation)
  - agimus_controller: ActuationModelFloatingBase / mixed actuation model
  - A base_cmd_bridge node that converts the planar base component of /control
    to geometry_msgs/Twist on /mobile_base_controller/cmd_vel

State space with planar base + 7-DoF right arm:
  nq = 4  (x, y, cos(yaw), sin(yaw))  +  7  (arm joints)  = 11
  nv = 3  (vx, vy, omega)             +  7  (arm joints)  = 10
"""

import numpy as np
import pinocchio
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.ros_utils import (
    weighted_traj_point_to_mpc_msg,
)
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase
from agimus_controller_ros.trajectory_weights_parameters import trajectory_weights_params


def _broadcast(v: list, size: int) -> list:
    """Broadcast a single-element list to `size` elements."""
    assert isinstance(v, list)
    return v * size if len(v) == 1 else v


class FixedGoalPublisher(TrajectoryPublisherBase):
    """
    Publishes a constant MpcInput that drives the EE to a fixed Cartesian target.

    The target SE3 pose is built from the parameters:
      goal_x, goal_y, goal_z  — position in base_footprint frame
      goal_roll, goal_pitch, goal_yaw — orientation (RPY, radians)

    The base is free to move (planar joint). The EE cost is applied only on
    position (orientation weights default to 0).
    """

    def __init__(self):
        self._dt = None
        self._publish_period = None
        super().__init__("fixed_goal_publisher")

        self._traj_weight_param_listener = trajectory_weights_params.ParamListener(self)
        self._traj_weight_params = self._traj_weight_param_listener.get_params()

        # Goal pose parameters
        self.declare_parameter("goal_x", 0.5)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.0)
        self.declare_parameter("goal_roll", 0.0)
        self.declare_parameter("goal_pitch", 0.0)
        self.declare_parameter("goal_yaw", 0.0)

        self._goal_x = self.get_parameter("goal_x").value
        self._goal_y = self.get_parameter("goal_y").value
        self._goal_z = self.get_parameter("goal_z").value
        self._goal_roll = self.get_parameter("goal_roll").value
        self._goal_pitch = self.get_parameter("goal_pitch").value
        self._goal_yaw = self.get_parameter("goal_yaw").value

        # ocp.dt and rate are fetched asynchronously via _agimus_params_to_fetch.

        # The visual servoing key must match ResidualModelVisualServoing.input_key
        self._vs_key = self._traj_weight_params.ee_frame_name + "_vs"

        self._goal_point: WeightedTrajectoryPoint | None = None
        self._msg_id = 0

    def _agimus_params_to_fetch(self) -> list:
        return ["free_flyer", "planar_base", "ocp.dt", "rate"]

    def _on_agimus_params(self, values) -> None:
        self._free_flyer = values[0].bool_value
        self._planar_base = values[1].bool_value
        self._dt = values[2].double_value
        self._publish_period = 1.0 / values[3].double_value

    @property
    def ee_frame_name(self) -> str:
        return self._traj_weight_params.ee_frame_name

    def ready_callback(self):
        """Build the constant goal trajectory point and start the timer."""
        model = self.robot_models.robot_model
        nv = model.nv

        w_q = _broadcast(list(self._traj_weight_params.w_q), nv)
        w_v = _broadcast(list(self._traj_weight_params.w_qdot), nv)
        w_a = _broadcast(list(self._traj_weight_params.w_qddot), nv)
        w_tau = _broadcast(list(self._traj_weight_params.w_robot_effort), nv)
        w_pose = _broadcast(list(self._traj_weight_params.w_pose), 6)

        # Build target SE3 from goal parameters
        rotation = pinocchio.utils.rpyToMatrix(
            self._goal_roll, self._goal_pitch, self._goal_yaw
        )
        translation = np.array([self._goal_x, self._goal_y, self._goal_z])
        goal_se3 = pinocchio.SE3(rotation, translation)

        # Publish 'universe → ee_target' as a static TF so that
        # ResidualModelVisualServoing can look up the transform.
        # ee_target is defined as the desired EE frame in the universe frame,
        # so oMf_target (below) is Identity: the EE should reach ee_target directly.
        self._publish_ee_target_tf(goal_se3)

        # Gravity torque at initial pose (used as lightweight feedforward regularisation)
        data = model.createData()
        tau_q0 = pinocchio.rnea(model, data, self.q0, np.zeros(nv), np.zeros(nv))

        self._goal_point = WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                id=0,
                robot_configuration=self.q0.copy(),
                robot_velocity=np.zeros(nv),
                robot_acceleration=np.zeros(nv),
                robot_effort=tau_q0.copy(),
                end_effector_poses={self._vs_key: pinocchio.SE3.Identity()},
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=w_q,
                w_robot_velocity=w_v,
                w_robot_acceleration=w_a,
                w_robot_effort=w_tau,
                w_end_effector_poses={self._vs_key: w_pose},
            ),
        )

        self.get_logger().info(
            f"FixedGoalPublisher ready. Target EE: "
            f"[{self._goal_x:.3f}, {self._goal_y:.3f}, {self._goal_z:.3f}] "
            f"frame={self.ee_frame_name}"
        )
        self.timer = self.create_timer(self._publish_period, self.publish_reference)

    def _publish_ee_target_tf(self, goal_se3: pinocchio.SE3) -> None:
        """Publish a static TF 'universe → ee_target' at the goal SE3 pose.

        ResidualModelVisualServoing looks up this transform to place the EE
        reference in the world frame. Since the goal is fixed, a static TF is
        sufficient. ee_target is defined such that the EE should reach it with
        identity relative pose (oMf_target = SE3.Identity()).
        """
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "universe"
        ts.child_frame_id = "ee_target"

        t = goal_se3.translation
        q = pinocchio.Quaternion(goal_se3.rotation)
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation.x = float(q.x)
        ts.transform.rotation.y = float(q.y)
        ts.transform.rotation.z = float(q.z)
        ts.transform.rotation.w = float(q.w)

        self._tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_static_broadcaster.sendTransform(ts)
        self.get_logger().info(
            f"Published static TF: universe → ee_target at "
            f"[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]"
        )

    def publish_reference(self):
        # Keep robot_configuration tracking the current state so the state
        # regularisation cost does not fight the EE tracking cost.
        self._goal_point.point.robot_configuration = self.current_q.copy()
        self._goal_point.point.id = self._msg_id
        self._msg_id += 1
        self.publisher_.publish(weighted_traj_point_to_mpc_msg(self._goal_point))


def main():
    rclpy.init()
    node = FixedGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
