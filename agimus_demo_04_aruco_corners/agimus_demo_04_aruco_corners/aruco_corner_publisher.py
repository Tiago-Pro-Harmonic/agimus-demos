"""
ArucoCornerPublisher node.

Drives fer_hand_tcp sequentially to hover 2 cm above each of the 4 corners of a
fixed ArUco marker, then returns the robot to its neutral configuration.

Corner ordering follows the OpenCV ArUco convention in the marker frame
(X right, Y up, Z pointing out toward the camera/robot):
    Corner 0  top-left:     [-h,  h, z_off]
    Corner 1  top-right:    [ h,  h, z_off]
    Corner 2  bottom-right: [ h, -h, z_off]
    Corner 3  bottom-left:  [-h, -h, z_off]

The node uses the ResidualModelVisualServoing interface: it publishes the corner
pose in the ArUco marker frame as the MpcInput EE target. The MPC controller reads
the TF "fer_link0 → aruco_marker" and computes the world-frame target at each solve
step:
    wMtarget = TF(fer_link0 → aruco_marker) * oMcorner

In simulation the TF is provided by a static_transform_publisher in the launch file.
On the real robot (use_aruco_detection:=true), this node subscribes to /aruco_markers
published by the ros2_aruco node and broadcasts the TF dynamically from detections,
latching the last known pose when the marker is temporarily out of view.
"""

import enum

import numpy as np
import pinocchio
import rclpy
from geometry_msgs.msg import TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from std_srvs.srv import Trigger
import tf2_ros

from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)
from agimus_controller_ros.ros_utils import (
    get_params_from_node,
    weighted_traj_point_to_mpc_msg,
)
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase
from agimus_controller_ros.trajectory_weights_parameters import trajectory_weights_params

# Franka Panda "ready" configuration (7 revolute joints).
# The robot returns here after visiting all 4 corners.
FRANKA_NEUTRAL_Q = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)


def _as_list_of_size(v: list, size: int) -> list:
    """Broadcast a single-element list to `size` elements; return multi-element lists as-is."""
    assert isinstance(v, list)
    return v * size if len(v) == 1 else v


class Phase(enum.IntEnum):
    CORNER_0 = 0
    CORNER_1 = 1
    CORNER_2 = 2
    CORNER_3 = 3
    NEUTRAL = 4
    DONE = 5


class ArucoCornerPublisher(TrajectoryPublisherBase):
    """
    Reference publisher for the ArUco corner pinpointing demo.

    State machine: CORNER_0 → CORNER_1 → CORNER_2 → CORNER_3 → NEUTRAL → DONE

    Each CORNER_i phase publishes MpcInput messages that drive fer_hand_tcp to hover
    above corner i.  After `dwell_time` seconds the state advances to the next corner.
    The NEUTRAL phase publishes messages with zero EE cost and q_ref = FRANKA_NEUTRAL_Q,
    letting the state regularisation cost bring the robot back to rest.
    """

    def __init__(self):
        self._dt = None
        super().__init__("aruco_corner_publisher")

        # Trajectory weights (ee_frame_name, w_q, w_qdot, w_qddot, w_robot_effort, w_pose)
        self._traj_weight_param_listener = trajectory_weights_params.ParamListener(self)
        self._traj_weight_params = self._traj_weight_param_listener.get_params()

        # Custom parameters — declared explicitly (not part of shared schema)
        self.declare_parameter("marker_size", 0.176)
        self.declare_parameter("approach_z_offset", 0.02)
        self.declare_parameter("dwell_time", 5.0)
        # Real-robot ArUco detection parameters
        self.declare_parameter("use_aruco_detection", False)
        self.declare_parameter("target_marker_id", 0)

        marker_size = self.get_parameter("marker_size").value
        approach_z = self.get_parameter("approach_z_offset").value
        self._dwell_time = self.get_parameter("dwell_time").value
        self._use_aruco_detection = self.get_parameter("use_aruco_detection").value
        self._target_marker_id = self.get_parameter("target_marker_id").value

        # Fetch the MPC timestep from agimus_controller_node
        params = get_params_from_node(self, "agimus_controller_node", ["ocp.dt"])
        self._dt = params[0].double_value

        # Visual servoing key: must match ResidualModelVisualServoing.input_key
        # which is built as robot_frame + "_vs" in ocp_croco_generic.py.
        self._vs_key = self._traj_weight_params.ee_frame_name + "_vs"

        # Corner poses in the marker frame.
        # Rotation = Identity: EE z-axis is aligned with the marker z-axis
        # (pointing back toward the robot), orientation is regulated only by
        # the state cost (w_pose orientation weights are zero).
        h = marker_size / 2.0
        self._corners = [
            pinocchio.SE3(np.eye(3), np.array([-h,  h, approach_z])),  # 0: top-left
            pinocchio.SE3(np.eye(3), np.array([ h,  h, approach_z])),  # 1: top-right
            pinocchio.SE3(np.eye(3), np.array([ h, -h, approach_z])),  # 2: bottom-right
            pinocchio.SE3(np.eye(3), np.array([-h, -h, approach_z])),  # 3: bottom-left
        ]

        # State machine initialisation
        self._phase = Phase.CORNER_0
        self._phase_elapsed = 0.0
        self._msg_id = 0

        # Populated in ready_callback once the robot model is available
        self._corner_points: list[WeightedTrajectoryPoint] = []
        self._neutral_point: WeightedTrajectoryPoint | None = None

    @property
    def ee_frame_name(self) -> str:
        return self._traj_weight_params.ee_frame_name

    # ------------------------------------------------------------------
    # Initialisation (called by TrajectoryPublisherBase once robot model
    # and q0 are available)
    # ------------------------------------------------------------------

    def ready_callback(self):
        """Build trajectory points and start the publishing timer."""
        while self._dt is None:
            self.get_logger().info("Waiting for agimus controller node params.")
            rclpy.spin_once(self, timeout_sec=1.0)

        model = self.robot_models.robot_model
        nv = model.nv

        assert nv == len(FRANKA_NEUTRAL_Q), (
            f"FRANKA_NEUTRAL_Q has {len(FRANKA_NEUTRAL_Q)} joints but model has {nv}."
        )

        # Weight vectors broadcast to the model dimension when needed
        w_q = _as_list_of_size(list(self._traj_weight_params.w_q), nv)
        w_v = _as_list_of_size(list(self._traj_weight_params.w_qdot), nv)
        w_a = _as_list_of_size(list(self._traj_weight_params.w_qddot), nv)
        w_tau = _as_list_of_size(list(self._traj_weight_params.w_robot_effort), nv)
        w_pose = _as_list_of_size(list(self._traj_weight_params.w_pose), 6)
        w_pose_zero = [0.0] * 6

        # ----- Corner trajectory points -----
        # robot_configuration is set to current_q at publish time (see publish_reference).
        # robot_effort is the gravity torque at the initial pose, kept constant; it is a
        # lightweight regularisation term (weight w_robot_effort is small).
        data = model.createData()
        tau_q0 = pinocchio.rnea(model, data, self.q0, np.zeros(nv), np.zeros(nv))

        for oMcorner in self._corners:
            pt = WeightedTrajectoryPoint(
                point=TrajectoryPoint(
                    id=0,
                    robot_configuration=self.q0.copy(),
                    robot_velocity=np.zeros(nv),
                    robot_acceleration=np.zeros(nv),
                    robot_effort=tau_q0.copy(),
                    end_effector_poses={self._vs_key: oMcorner},
                ),
                weights=TrajectoryPointWeights(
                    w_robot_configuration=w_q,
                    w_robot_velocity=w_v,
                    w_robot_acceleration=w_a,
                    w_robot_effort=w_tau,
                    w_end_effector_poses={self._vs_key: w_pose},
                ),
            )
            self._corner_points.append(pt)

        # ----- Neutral trajectory point -----
        # EE cost is zeroed out; state regularisation drives the robot to
        # FRANKA_NEUTRAL_Q. We must still provide the vs_key in end_effector_poses
        # because ResidualModelVisualServoing asserts exactly one EE entry exists.
        # With all-zero weights the transform lookup is skipped and the cost is zero.
        q_neutral = FRANKA_NEUTRAL_Q.copy()
        data_neutral = model.createData()
        tau_neutral = pinocchio.rnea(
            model, data_neutral, q_neutral, np.zeros(nv), np.zeros(nv)
        )
        self._neutral_point = WeightedTrajectoryPoint(
            point=TrajectoryPoint(
                id=0,
                robot_configuration=q_neutral,
                robot_velocity=np.zeros(nv),
                robot_acceleration=np.zeros(nv),
                robot_effort=tau_neutral,
                # Placeholder (weight is zero, value has no effect on the cost)
                end_effector_poses={self._vs_key: self._corners[3]},
            ),
            weights=TrajectoryPointWeights(
                w_robot_configuration=w_q,
                w_robot_velocity=w_v,
                w_robot_acceleration=w_a,
                w_robot_effort=w_tau,
                w_end_effector_poses={self._vs_key: w_pose_zero},
            ),
        )

        if self._use_aruco_detection:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self._last_aruco_tf: TransformStamped | None = None
            self.create_subscription(ArucoMarkers, "/aruco_markers", self._aruco_callback, 10)
            self.get_logger().info(
                f"ArUco detection enabled — subscribing to /aruco_markers "
                f"for marker ID {self._target_marker_id}."
            )

        self._next_corner_srv = self.create_service(
            Trigger, "~/next_corner", self._next_corner_callback
        )
        self.timer = self.create_timer(self._dt, self.publish_reference)
        self.get_logger().info(
            f"ArucoCornerPublisher ready.  "
            f"Will visit 4 corners ({self._dwell_time:.1f} s each, or call "
            f"~/next_corner service to advance manually) then return to neutral."
        )

    # ------------------------------------------------------------------
    # ArUco detection → TF broadcast (real robot only)
    # ------------------------------------------------------------------

    def _aruco_callback(self, msg: ArucoMarkers):
        """Broadcast camera_frame → aruco_marker TF from live detections.

        The parent frame is taken from the message header (set by the aruco_node
        to the camera optical frame). The last known pose is latched so the MPC
        keeps a valid target even when the marker is momentarily out of view.
        """
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id != self._target_marker_id:
                continue
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = "aruco_marker"
            p = msg.poses[i]
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation = p.orientation
            self._last_aruco_tf = t
            self._tf_broadcaster.sendTransform(t)
            return

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _advance_phase(self):
        """Move to the next phase and reset the dwell timer."""
        self._phase_elapsed = 0.0

        if self._phase == Phase.CORNER_3:
            self._phase = Phase.NEUTRAL
            self.get_logger().info("All 4 corners visited — returning to neutral.")
        elif self._phase == Phase.NEUTRAL:
            self._phase = Phase.DONE
            self.get_logger().info("Robot at neutral. Demo complete.")
        else:
            next_phase = Phase(self._phase + 1)
            self._phase = next_phase
            self.get_logger().info(f"Advancing to corner {self._phase.value}.")

    def _next_corner_callback(self, request, response):
        """Service handler: immediately advance to the next corner (or neutral/done)."""
        if self._phase == Phase.DONE:
            response.success = False
            response.message = "Demo already complete."
        else:
            prev = self._phase.name
            self._advance_phase()
            response.success = True
            response.message = f"Advanced from {prev} to {self._phase.name}."
        return response

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish_reference(self):
        # Re-broadcast the latched TF so the MPC never loses the marker transform
        # when it temporarily goes out of camera view.
        if self._use_aruco_detection and self._last_aruco_tf is not None:
            self._last_aruco_tf.header.stamp = self.get_clock().now().to_msg()
            self._tf_broadcaster.sendTransform(self._last_aruco_tf)

        # In DONE phase: keep publishing neutral so the MPC buffer never empties.
        if self._phase == Phase.DONE:
            self._neutral_point.point.id = self._msg_id
            self._msg_id += 1
            self.publisher_.publish(weighted_traj_point_to_mpc_msg(self._neutral_point))
            return

        self._phase_elapsed += self._dt
        if self._dwell_time > 0 and self._phase_elapsed >= self._dwell_time:
            self._advance_phase()

        if self._phase in (Phase.NEUTRAL, Phase.DONE):
            # Do NOT update robot_configuration: keep it at FRANKA_NEUTRAL_Q so
            # the state regularisation cost pulls toward the rest pose.
            pt = self._neutral_point
        else:
            # Track current joint state so the state cost doesn't fight the EE cost.
            pt = self._corner_points[self._phase.value]
            pt.point.robot_configuration = self.current_q.copy()

        pt.point.id = self._msg_id
        self._msg_id += 1

        msg = weighted_traj_point_to_mpc_msg(pt)
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = ArucoCornerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
