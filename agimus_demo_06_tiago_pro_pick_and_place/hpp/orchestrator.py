"""
HPP pick-and-place orchestrator for TIAGo Pro.

Provides an interactive interface for step-by-step planning and execution:
    o = Orchestrator()
    o.plan()             # run HPP planner
    o.execute()          # sample + publish trajectory to MPC
    o.plan_and_execute() # both in sequence

Run via orchestrator_node.py (sources both ros2_config.sh and hpp_config.sh).
"""

import os
import sys
import glob
import time
import numpy as np
import pinocchio as pin
import yaml

# ── Make agimus_msgs / rclpy importable from HPP environment ──────────────────
for _p in sorted(
    glob.glob("/home/user/devel/ros2_ws/install/*/lib/python3.12/site-packages")
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pyhpp.manipulation import Device, urdf
from pyhpp.manipulation import Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from agimus_msgs.msg import MpcInput
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


# ── Constants ─────────────────────────────────────────────────────────────────

_HPP_DIR   = os.path.dirname(os.path.abspath(__file__))
ROBOT_SRDF = os.path.join(_HPP_DIR, "tiago_pro.srdf")
BOX_SRDF   = os.path.join(_HPP_DIR, "box.srdf")
_CFG_FILE  = os.path.join(_HPP_DIR, "..", "config", "hpp_orchestrator_params.yaml")

with open(_CFG_FILE) as _f:
    _cfg = yaml.safe_load(_f)

# Trajectory
DT = _cfg["trajectory"]["dt"]

# Scene geometry
_s              = _cfg["scene"]
TABLE_OFFSET_X  = _s["table_offset_x"]
TABLE_OFFSET_Z  = _s["table_offset_z"]
TABLE_Z         = 0.730 + TABLE_OFFSET_Z
BOX_HALF        = 0.025
BOX_Z           = TABLE_Z + BOX_HALF
TABLE_NEAR_EDGE = 0.8 + TABLE_OFFSET_X - 0.3
BOX_X           = TABLE_NEAR_EDGE + BOX_HALF + 0.02
BOX_Y_INIT      = _s["box_y_init"]
BOX_Y_GOAL      = _s["box_y_goal"]

# Tuck poses
LEFT_ARM_TUCK  = _cfg["tuck"]["left_arm"]
RIGHT_ARM_TUCK = _cfg["tuck"]["right_arm"]

# MpcInput weights (nv=10: 3 base + 7 arm)
_w       = _cfg["weights"]
W_Q         = np.array(_w["w_q"])
W_QDOT      = np.array(_w["w_qdot"])
W_QDDOT     = np.array(_w["w_qddot"])
W_EFFORT    = np.array(_w["w_effort"])
W_COLLISION = _w["w_collision"]


class _TrajectoryPublisherNode(Node):
    """One-shot ROS2 node that publishes pre-computed MpcInput messages."""

    def __init__(self, messages: list):
        super().__init__("hpp_trajectory_publisher")
        self._messages = messages
        self._idx = 0
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(MpcInput, "mpc_input", qos)
        self._timer = self.create_timer(DT, self._publish_next)
        self.get_logger().info(
            f"Publishing {len(self._messages)} trajectory points at {1/DT:.0f} Hz …"
        )
        self._done = False

    def _publish_next(self):
        if self._idx >= len(self._messages):
            if not self._done:
                self.get_logger().info("Trajectory fully published.")
                self._done = True
            self._timer.cancel()
            return
        self._pub.publish(self._messages[self._idx])
        self._idx += 1


class Orchestrator:
    """
    Interactive orchestrator for HPP pick-and-place planning and MPC execution.

    Usage (in IPython):
        o = Orchestrator()
        o.plan()
        o.execute()
    """

    def __init__(self, ros_node: Node = None):
        self._ros_node = ros_node
        self._hpp_path = None
        self._messages = None

        print("Loading HPP model …")
        self._setup_model()
        print("Building constraint graph …")
        self._setup_graph()
        print("Orchestrator ready.  Call plan() to start.\n")

    # ── Model setup ───────────────────────────────────────────────────────────

    def _setup_model(self):
        robot = Device("tiago_pro")
        urdf.loadModel(
            robot, 0, "tiago_pro", "planar",
            "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf",
            ROBOT_SRDF,
            pin.SE3.Identity(),
        )
        urdf.loadModel(
            robot, 0, "table", "anchor",
            "package://hpp_tutorial/urdf/table.urdf",
            "package://hpp_tutorial/srdf/table.srdf",
            pin.SE3(np.eye(3), np.array([TABLE_OFFSET_X, 0, TABLE_OFFSET_Z])),
        )
        urdf.loadModel(
            robot, 0, "ground", "anchor",
            "package://hpp_tutorial/urdf/ground.urdf",
            "package://hpp_tutorial/srdf/ground.srdf",
            pin.SE3.Identity(),
        )
        urdf.loadModel(
            robot, 0, "box", "freeflyer",
            "package://hpp_tutorial/urdf/box.urdf",
            BOX_SRDF,
            pin.SE3.Identity(),
        )

        self.robot = robot
        model = robot.model()
        self.model = model

        def _idx(name):
            jid = model.getJointId(name)
            return model.joints[jid].idx_q

        self._base_idx      = _idx("tiago_pro/root_joint")
        self._left_arm_idx  = _idx("tiago_pro/arm_left_1_joint")
        self._right_arm_idx = _idx("tiago_pro/arm_right_1_joint")
        self._box_idx       = _idx("box/root_joint")

        robot.setJointBounds("tiago_pro/root_joint", [
            -2.0, 2.0, -2.0, 2.0,
            -float("Inf"), float("Inf"), -float("Inf"), float("Inf"),
        ])
        robot.setJointBounds("box/root_joint", [
            -1.5, 3.0, -1.5, 1.5, -0.2, 1.5,
            -float("Inf"), float("Inf"), -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"), -float("Inf"), float("Inf"),
        ])

        self._disable_cross_model_collisions()

        li = self._left_arm_idx
        ri = self._right_arm_idx
        xi = self._box_idx

        self.q_init = pin.neutral(model).copy()
        self.q_init[li:li+7] = LEFT_ARM_TUCK
        self.q_init[ri:ri+7] = RIGHT_ARM_TUCK
        self.q_init[xi:xi+7] = [BOX_X, BOX_Y_INIT, BOX_Z, 0., 0., 0., 1.]

        self.q_goal = pin.neutral(model).copy()
        self.q_goal[li:li+7] = LEFT_ARM_TUCK
        self.q_goal[ri:ri+7] = RIGHT_ARM_TUCK
        self.q_goal[xi:xi+7] = [BOX_X, BOX_Y_GOAL, BOX_Z, 0., 0., 0., 1.]

    def _disable_cross_model_collisions(self):
        """Disable known cross-model collision pairs.

        Intra-robot collisions are handled by tiago_pro.srdf.
        Cross-model contacts (table vs box, ground vs robot, etc.) span
        separate SRDF files and must be disabled explicitly here.
        """
        geom = self.robot.geomModel()

        def _disable(name1, name2):
            ids1 = [i for i, o in enumerate(geom.geometryObjects) if name1 in o.name]
            ids2 = [i for i, o in enumerate(geom.geometryObjects) if name2 in o.name]
            removed = 0
            for i in ids1:
                for j in ids2:
                    cp = pin.CollisionPair(min(i, j), max(i, j))
                    idx = geom.findCollisionPair(cp)
                    if idx < len(geom.collisionPairs):
                        geom.removeCollisionPair(geom.collisionPairs[idx])
                        removed += 1
            return removed

        # Box resting on table — expected contact, not a collision
        n = _disable("pancake_table_table_top_link", "box/base_link")
        print(f"  {n} cross-model pair(s) disabled, "
              f"{len(geom.collisionPairs)} remaining.")

    # ── Constraint graph setup ─────────────────────────────────────────────────

    def _setup_graph(self):
        robot = self.robot
        model = self.model

        problem = Problem(robot)
        graph   = Graph("robot", robot, problem)
        factory = ConstraintGraphFactory(graph)
        graph.maxIterations(40)
        graph.errorThreshold(1e-3)
        factory.setGrippers(["tiago_pro/gripper"])
        factory.setObjects(["box"], [["box/handle"]], [["box/surface"]])
        factory.environmentContacts(["table/pancake_table_table_top"])
        factory.generate()

        _cts = ComparisonTypes()
        _cts[:] = [ComparisonType.EqualToZero]

        def _lock(joint_name, value):
            jid = model.getJointId(joint_name)
            j   = model.joints[jid]
            if j.nq == 2 and j.nv == 1:
                locked_val = np.array([np.cos(value), np.sin(value)])
            else:
                locked_val = np.array([value])
            return LockedJoint(robot, joint_name, locked_val, _cts)

        locked = []
        locked.append(_lock("tiago_pro/torso_lift_joint", 0.0))
        for wheel in ["wheel_front_left_joint", "wheel_front_right_joint",
                      "wheel_rear_left_joint",  "wheel_rear_right_joint"]:
            locked.append(_lock(f"tiago_pro/{wheel}", 0.0))
        for i, val in enumerate(LEFT_ARM_TUCK):
            locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))
        for name in ["gripper_left_finger_joint",
                     "gripper_left_inner_finger_left_joint",
                     "gripper_left_fingertip_left_joint",
                     "gripper_left_inner_finger_right_joint",
                     "gripper_left_fingertip_right_joint",
                     "gripper_left_outer_finger_right_joint"]:
            locked.append(_lock(f"tiago_pro/{name}", 0.0))
        for name in ["gripper_right_finger_joint",
                     "gripper_right_inner_finger_left_joint",
                     "gripper_right_fingertip_left_joint",
                     "gripper_right_inner_finger_right_joint",
                     "gripper_right_fingertip_right_joint",
                     "gripper_right_outer_finger_right_joint"]:
            locked.append(_lock(f"tiago_pro/{name}", 0.0))
        locked.append(_lock("tiago_pro/head_1_joint", 0.0))
        locked.append(_lock("tiago_pro/head_2_joint", 0.0))

        graph.addNumericalConstraintsToGraph(locked)
        graph.initialize()

        self.problem = problem
        self.graph   = graph

    # ── Planning ──────────────────────────────────────────────────────────────

    def plan(self, max_iter: int = 10000) -> bool:
        """Run the HPP manipulation planner. Returns True on success."""
        self.problem.initConfig(self.q_init)
        self.problem.addGoalConfig(self.q_goal)
        self.problem.constraintGraph(self.graph)

        planner = ManipulationPlanner(self.problem)
        planner.maxIterations(max_iter)

        print(f"Planning (max {max_iter} iterations) …")
        try:
            self._hpp_path = planner.solve()
            tr = self._hpp_path.timeRange()
            print(f"  Path found!  Duration: {tr.second - tr.first:.2f} s")
        except RuntimeError as e:
            print(f"  Planner failed: {e}")
            self._hpp_path = None
            return False

        print("Optimising path with RandomShortcut …")
        try:
            optimizer = RandomShortcut(self.problem)
            path_opt = optimizer.optimize(self._hpp_path)
            tr_opt = path_opt.timeRange()
            print(f"  Optimised!  Duration: {tr_opt.second - tr_opt.first:.2f} s  "
                  f"(was {tr.second - tr.first:.2f} s)")
            self._hpp_path = path_opt
        except Exception as e:
            print(f"  Optimiser failed: {e} — keeping original path.")

        return True

    # ── Path sampling ─────────────────────────────────────────────────────────

    def _extract_active_q(self, q_full):
        q = np.array(q_full)
        bi = self._base_idx
        ri = self._right_arm_idx
        return np.concatenate([q[bi:bi+4], q[ri:ri+7]])

    def _active_velocity(self, q1, q2, dt):
        vx = (q2[0] - q1[0]) / dt
        vy = (q2[1] - q1[1]) / dt
        theta1 = np.arctan2(q1[3], q1[2])
        theta2 = np.arctan2(q2[3], q2[2])
        dtheta = (theta2 - theta1 + np.pi) % (2 * np.pi) - np.pi
        omega  = dtheta / dt
        d_arm  = (q2[4:] - q1[4:]) / dt
        return np.concatenate([[vx, vy, omega], d_arm])

    def _sample_path(self, path, dt=DT):
        tr = path.timeRange()
        t_min, t_max = tr.first, tr.second
        times = np.arange(t_min, t_max, dt)
        if len(times) == 0:
            return None, None, None

        q_list = [self._extract_active_q(path.eval(t)[0]) for t in times]
        q_arr  = np.array(q_list)

        dq_list = [self._active_velocity(q_arr[i], q_arr[i+1], dt)
                   for i in range(len(q_arr) - 1)]
        dq_list.append(dq_list[-1])
        dq_arr = np.array(dq_list)

        ddq_list = [(dq_arr[i+1] - dq_arr[i]) / dt
                    for i in range(len(dq_arr) - 1)]
        ddq_list.append(ddq_list[-1])
        ddq_arr = np.array(ddq_list)

        return q_arr, dq_arr, ddq_arr

    def _build_msg(self, q, dq, ddq, msg_id):
        msg = MpcInput()
        msg.id        = msg_id
        msg.q         = q.tolist()
        msg.qdot      = dq.tolist()
        msg.qddot     = ddq.tolist()
        msg.robot_effort = np.zeros(10).tolist()

        msg.w_q            = W_Q.tolist()
        msg.w_qdot         = W_QDOT.tolist()
        msg.w_qddot        = W_QDDOT.tolist()
        msg.w_robot_effort = W_EFFORT.tolist()
        msg.w_collision_avoidance = W_COLLISION
        return msg

    def _build_messages(self) -> list:
        print("Sampling path …")
        q_arr, dq_arr, ddq_arr = self._sample_path(self._hpp_path)
        print(f"  {len(q_arr)} waypoints at dt={DT} s "
              f"(total {len(q_arr)*DT:.1f} s)")

        msgs = [
            self._build_msg(q, dq, ddq, i)
            for i, (q, dq, ddq) in enumerate(zip(q_arr, dq_arr, ddq_arr))
        ]
        print(f"  {len(msgs)} MpcInput messages ready.")
        return msgs

    # ── Execution ─────────────────────────────────────────────────────────────

    def execute(self):
        """Sample the planned path and publish MpcInput messages to the controller."""
        if self._hpp_path is None:
            print("No path available — run plan() first.")
            return

        self._messages = self._build_messages()

        if self._ros_node is None:
            self._ros_node = _TrajectoryPublisherNode(self._messages)
        else:
            self._ros_node._messages = self._messages
            self._ros_node._idx      = 0
            self._ros_node._done     = False
            self._ros_node._timer    = self._ros_node.create_timer(
                DT, self._ros_node._publish_next
            )

        print("Publishing trajectory …")
        try:
            while not self._ros_node._done:
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                time.sleep(DT)
        except KeyboardInterrupt:
            print("\nExecution interrupted.")

    # ── Visualisation ─────────────────────────────────────────────────────────

    def init_viewer(self, open: bool = True):
        """Open a Viser viewer. Call once, then use o.view(q) or o.visualize()."""
        from pyhpp_viser import Viewer
        self._viewer = Viewer(self.robot)
        self._viewer.initViewer(open=open, loadModel=True)
        self._viewer.setProblem(self.problem)
        self._viewer.setGraph(self.graph)
        self._viewer(self.q_init)
        print("Viser viewer ready.  Use o.view(q) or o.visualize().")

    def view(self, q=None):
        """Display a configuration in Viser (defaults to q_init)."""
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        self._viewer(q if q is not None else self.q_init)

    def visualize(self):
        """Play the planned path in Viser."""
        if self._hpp_path is None:
            print("No path — run plan() first.")
            return
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        self._viewer.loadPath(self._hpp_path)
        print("Path loaded in Viser.")

    # ── Robot state sync ──────────────────────────────────────────────────────

    def sync_from_robot(self, timeout: float = 5.0):
        """Update q_init from the current Gazebo robot state.

        Reads /joint_states (arm positions) and /odom (base pose) and updates
        self.q_init so that HPP planning starts from the actual robot state.
        """
        if self._ros_node is None:
            self._ros_node = rclpy.create_node("hpp_sync_node")
            _own_node = True
        else:
            _own_node = False

        joint_state   = [None]
        odom_state    = [None]

        joint_sub = self._ros_node.create_subscription(
            JointState, "/joint_states",
            lambda msg: joint_state.__setitem__(0, msg), 10
        )
        odom_sub = self._ros_node.create_subscription(
            Odometry, "/odom",
            lambda msg: odom_state.__setitem__(0, msg), 10
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            if joint_state[0] is not None and odom_state[0] is not None:
                break

        self._ros_node.destroy_subscription(joint_sub)
        self._ros_node.destroy_subscription(odom_sub)
        if _own_node:
            self._ros_node.destroy_node()
            self._ros_node = None

        if joint_state[0] is None or odom_state[0] is None:
            missing = []
            if joint_state[0] is None:
                missing.append("/joint_states")
            if odom_state[0] is None:
                missing.append("/odom")
            print(f"sync_from_robot: timeout — could not receive {', '.join(missing)}")
            return

        # ── Extract base pose from /odom ──────────────────────────────────────
        odom = odom_state[0]
        bx   = odom.pose.pose.position.x
        by   = odom.pose.pose.position.y
        q_odom = odom.pose.pose.orientation
        # Convert quaternion (x,y,z,w) to yaw, then store as [cos θ, sin θ]
        siny_cosp = 2.0 * (q_odom.w * q_odom.z + q_odom.x * q_odom.y)
        cosy_cosp = 1.0 - 2.0 * (q_odom.y * q_odom.y + q_odom.z * q_odom.z)
        theta     = np.arctan2(siny_cosp, cosy_cosp)

        # ── Extract arm joint positions from /joint_states ───────────────────
        js     = joint_state[0]
        js_map = dict(zip(js.name, js.position))

        def _read_arm(side):
            joints = [f"arm_{side}_{i}_joint" for i in range(1, 8)]
            return np.array([
                js_map.get(f"tiago_pro/{j}", js_map.get(j, 0.0))
                for j in joints
            ])

        right_arm_q = _read_arm("right")
        left_arm_q  = _read_arm("left")

        # ── Update q_init ─────────────────────────────────────────────────────
        bi = self._base_idx
        ri = self._right_arm_idx
        li = self._left_arm_idx
        self.q_init[bi]     = bx
        self.q_init[bi + 1] = by
        self.q_init[bi + 2] = np.cos(theta)
        self.q_init[bi + 3] = np.sin(theta)
        self.q_init[ri:ri+7] = right_arm_q
        self.q_init[li:li+7] = left_arm_q

        print(f"sync_from_robot: base=({bx:.3f}, {by:.3f}, θ={np.degrees(theta):.1f}°)")
        print(f"  right arm: {np.round(right_arm_q, 3).tolist()}")
        print(f"  left  arm: {np.round(left_arm_q,  3).tolist()}")

    # ── Combined ──────────────────────────────────────────────────────────────

    def plan_and_execute(self, max_iter: int = 10000):
        """Plan then immediately execute."""
        if self.plan(max_iter=max_iter):
            self.execute()
