"""
HPP deburring orchestrator for TIAGo Pro.

Provides an interactive interface for step-by-step planning and execution:
    o = Orchestrator()
    o.plan()             # run HPP planner (generates qpg, qg, p1, p2, p3)
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
from pyhpp.manipulation import Graph, Problem, TransitionPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut, SplineGradientBased_bezier3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from agimus_msgs.msg import MpcInput
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


# ── Constants ─────────────────────────────────────────────────────────────────

_HPP_DIR    = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR    = os.path.join(_HPP_DIR, "..")
ROBOT_SRDF  = os.path.join(_HPP_DIR, "tiago_pro.srdf")
PYLONE_SRDF = os.path.join(_HPP_DIR, "pylone.srdf")
PYLONE_URDF = os.path.join(_PKG_DIR, "urdf", "pylone.urdf")
TABLE_SRDF  = os.path.join(_HPP_DIR, "table.srdf")
TABLE_URDF  = os.path.join(_PKG_DIR, "urdf", "table.urdf")
GROUND_SRDF = os.path.join(_HPP_DIR, "ground.srdf")
GROUND_URDF = os.path.join(_PKG_DIR, "urdf", "ground.urdf")

_CFG_FILE = os.path.join(_PKG_DIR, "config", "hpp_orchestrator_params.yaml")
with open(_CFG_FILE) as _f:
    _cfg = yaml.safe_load(_f)

DT         = _cfg["trajectory"]["dt"]
TIME_SCALE = _cfg["trajectory"]["time_scale"]

_s        = _cfg["scene"]
TABLE_X   = _s["table_x"]
TABLE_Y   = _s["table_y"]
TABLE_Z   = _s["table_z"]
PYLONE_X  = TABLE_X
PYLONE_Y  = TABLE_Y
PYLONE_Z  = _s["pylone_z_offset"]

_h           = _cfg["handle"]
HANDLE_LINK  = _h["link"]
HANDLE_NAME  = _h["name"]
HANDLE_POS   = np.array(_h["position"])
HANDLE_CLEAR = _h["clearance"]

LEFT_ARM_TUCK  = _cfg["tuck"]["left_arm"]
RIGHT_ARM_TUCK = _cfg["tuck"]["right_arm"]

_w          = _cfg["weights"]
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
    Interactive orchestrator for HPP deburring planning and MPC execution.

    Usage (in IPython):
        o = Orchestrator()
        o.plan()
        o.execute()
    """

    def __init__(self, ros_node: Node = None):
        self._ros_node = ros_node
        self.p1 = self.p2 = self.p3 = None
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
            robot, 0, "ground", "anchor",
            GROUND_URDF,
            GROUND_SRDF,
            pin.SE3.Identity(),
        )
        urdf.loadModel(
            robot, 0, "table", "anchor",
            TABLE_URDF,
            TABLE_SRDF,
            pin.SE3(np.eye(3), np.array([TABLE_X, TABLE_Y, TABLE_Z])),
        )
        urdf.loadModel(
            robot, 0, "pylone", "freeflyer",
            PYLONE_URDF,
            PYLONE_SRDF,
            pin.SE3.Identity(),
        )

        self.robot = robot
        model = robot.model()
        self.model = model

        def _idx(name):
            return model.joints[model.getJointId(name)].idx_q

        self._base_idx      = _idx("tiago_pro/root_joint")
        self._left_arm_idx  = _idx("tiago_pro/arm_left_1_joint")
        self._right_arm_idx = _idx("tiago_pro/arm_right_1_joint")
        self._pylone_idx    = _idx("pylone/root_joint")

        robot.setJointBounds("tiago_pro/root_joint", [
            -3.0, 3.0, -3.0, 3.0,
            -float("Inf"), float("Inf"), -float("Inf"), float("Inf"),
        ])
        robot.setJointBounds("pylone/root_joint", [
            PYLONE_X - 0.01, PYLONE_X + 0.01,
            PYLONE_Y - 0.01, PYLONE_Y + 0.01,
            PYLONE_Z - 0.01, PYLONE_Z + 0.01,
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
            -float("Inf"), float("Inf"),
        ])

        # Add handle: Rx(-90°) so handle Z = world +Y (into the hole)
        _R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        robot.addHandle(
            HANDLE_LINK, HANDLE_NAME,
            pin.SE3(_R, HANDLE_POS),
            HANDLE_CLEAR,
            6 * [True],
        )
        robot.handles()[HANDLE_NAME].approachingDirection = np.array([0, 0, 1])

        li = self._left_arm_idx
        ri = self._right_arm_idx
        pi = self._pylone_idx

        self.q_init = pin.neutral(model).copy()
        self._left_arm_lock_values = list(LEFT_ARM_TUCK)
        self.q_init[li:li+7] = LEFT_ARM_TUCK
        self.q_init[ri:ri+7] = RIGHT_ARM_TUCK
        self.q_init[pi:pi+7] = [PYLONE_X, PYLONE_Y, PYLONE_Z, 0., 0., 0., 1.]

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
        factory.setObjects(["pylone"], [[HANDLE_NAME]], [[]])
        factory.generate()

        self._transition_approach = graph.getTransition(
            f"tiago_pro/gripper > {HANDLE_NAME} | f_01"
        )
        self._transition_insert = graph.getTransition(
            f"tiago_pro/gripper > {HANDLE_NAME} | f_12"
        )

        _cts = ComparisonTypes()
        _cts[:] = [ComparisonType.EqualToZero]

        def _lock(joint_name, value):
            j = model.joints[model.getJointId(joint_name)]
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
        for i, val in enumerate(self._left_arm_lock_values):
            locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))
        for name in ["gripper_left_finger_joint",
                     "gripper_left_inner_finger_left_joint",
                     "gripper_left_fingertip_left_joint",
                     "gripper_left_inner_finger_right_joint",
                     "gripper_left_fingertip_right_joint",
                     "gripper_left_outer_finger_right_joint",
                     "gripper_right_finger_joint",
                     "gripper_right_inner_finger_left_joint",
                     "gripper_right_fingertip_left_joint",
                     "gripper_right_inner_finger_right_joint",
                     "gripper_right_fingertip_right_joint",
                     "gripper_right_outer_finger_right_joint"]:
            locked.append(_lock(f"tiago_pro/{name}", 0.0))
        locked.append(_lock("tiago_pro/head_1_joint", 0.0))
        locked.append(_lock("tiago_pro/head_2_joint", 0.0))

        graph.addNumericalConstraintsToGraph(locked)

        sm = SecurityMargins(problem, factory, ["tiago_pro", "pylone"], robot)
        sm.setSecurityMarginBetween("tiago_pro", "pylone", 0.05)
        sm.setSecurityMarginBetween("tiago_pro", "table", 0.05)
        sm.apply()

        for jname in model.names:
            if "tiago_pro" in jname:
                graph.setSecurityMarginForTransition(
                    self._transition_insert, jname, "pylone/root_joint", float("-inf")
                )

        graph.initialize()

        self.problem = problem
        self.graph   = graph

    # ── Planning ──────────────────────────────────────────────────────────────

    def plan(self, max_attempts: int = 50) -> bool:
        """Generate qpg (collision-free), qg, and plan p1, p2, p3."""
        shooter = self.problem.configurationShooter()
        qpg = None
        for i in range(max_attempts):
            q = shooter.shoot()
            res, q_cand, err = self.graph.generateTargetConfig(
                self._transition_approach, self.q_init, q
            )
            if not res:
                continue
            pv = self._transition_approach.pathValidation()
            res, _ = pv.validateConfiguration(q_cand)
            if not res:
                continue
            qpg = q_cand
            print(f"  qpg found at attempt {i}, err={err:.2e}")
            break

        if qpg is None:
            print(f"Failed to find collision-free qpg in {max_attempts} attempts.")
            return False

        res, qg, err = self.graph.generateTargetConfig(
            self._transition_insert, qpg, qpg
        )
        print(f"  qg: res={res}, err={err:.2e}")
        if not res:
            print("Failed to generate qg.")
            return False

        self.problem.constraintGraph(self.graph)
        planner = TransitionPlanner(self.problem)
        planner.maxIterations(1000)

        planner.setTransition(self._transition_approach)
        q_goal = np.zeros((1, self.robot.configSize()), order='F')
        q_goal[0, :] = qpg
        print("Planning p1 (approach) …")
        p1 = planner.planPath(self.q_init, q_goal, True)
        print("  p1 found.")

        shortcut   = RandomShortcut(self.problem)
        spline_opt = SplineGradientBased_bezier3(self.problem)

        # p1: shortcut first, then smooth with Bezier splines
        try:
            for i in range(3):
                p1_new = shortcut.optimize(p1)
                tr_before = p1.timeRange()
                tr_after  = p1_new.timeRange()
                dt = (tr_before.second - tr_before.first) - (tr_after.second - tr_after.first)
                p1 = p1_new
                print(f"  p1 shortcut pass {i+1}/3: {tr_after.second - tr_after.first:.2f} s  (−{dt:.2f} s)")
                if dt < 1e-3:
                    break
        except Exception as e:
            print(f"  p1 shortcut failed: {e}")
        try:
            p1 = spline_opt.optimize(p1)
            tr = p1.timeRange()
            print(f"  p1 spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p1 spline optimisation failed: {e}")

        planner.setTransition(self._transition_insert)
        q_goal[0, :] = qg
        print("Planning p2 (insertion) …")
        p2 = planner.planPath(qpg, q_goal, True)
        print("  p2 found.")

        # p2: same pipeline
        try:
            p2 = shortcut.optimize(p2)
            tr = p2.timeRange()
            print(f"  p2 shortcut: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p2 shortcut failed: {e}")
        try:
            p2 = spline_opt.optimize(p2)
            tr = p2.timeRange()
            print(f"  p2 spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            print(f"  p2 spline optimisation failed: {e}")

        p3 = p2.reverse()
        print("  p3 ready (retraction, reversed from optimised p2).")

        self.p1  = p1
        self.p2  = p2
        self.p3  = p3
        self.qpg = qpg
        self.qg  = qg
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

    def _sample_path(self, path):
        tr = path.timeRange()
        t_min, t_max = tr.first, tr.second
        n     = max(2, int((t_max - t_min) * TIME_SCALE / DT))
        times = np.linspace(t_min, t_max, n)
        q_list = [self._extract_active_q(path.eval(t)[0]) for t in times]
        q_arr  = np.array(q_list)

        dq_list = [self._active_velocity(q_arr[i], q_arr[i+1], DT)
                   for i in range(len(q_arr) - 1)]
        dq_list.append(dq_list[-1])
        dq_arr = np.array(dq_list)

        ddq_list = [(dq_arr[i+1] - dq_arr[i]) / DT
                    for i in range(len(dq_arr) - 1)]
        ddq_list.append(ddq_list[-1])
        ddq_arr = np.array(ddq_list)

        return q_arr, dq_arr, ddq_arr

    def _build_msg(self, q, dq, ddq, msg_id):
        msg = MpcInput()
        msg.id           = msg_id
        msg.q            = q.tolist()
        msg.qdot         = dq.tolist()
        msg.qddot        = ddq.tolist()
        msg.robot_effort = np.zeros(10).tolist()
        msg.w_q                   = W_Q.tolist()
        msg.w_qdot                = W_QDOT.tolist()
        msg.w_qddot               = W_QDDOT.tolist()
        msg.w_robot_effort        = W_EFFORT.tolist()
        msg.w_collision_avoidance = W_COLLISION
        return msg

    def _build_messages(self) -> list:
        msgs = []
        idx = 0
        for name, path in [("p1 (approach)", self.p1),
                            ("p2 (insertion)", self.p2),
                            ("p3 (retraction)", self.p3)]:
            q_arr, dq_arr, ddq_arr = self._sample_path(path)
            print(f"  {name}: {len(q_arr)} waypoints")
            for q, dq, ddq in zip(q_arr, dq_arr, ddq_arr):
                msgs.append(self._build_msg(q, dq, ddq, idx))
                idx += 1
        print(f"  {len(msgs)} MpcInput messages total.")
        return msgs

    # ── Execution ─────────────────────────────────────────────────────────────

    def execute(self):
        """Sample p1+p2+p3 and publish MpcInput messages to the controller."""
        if self.p1 is None:
            print("No path available — run plan() first.")
            return

        print("Sampling trajectories …")
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
        from pyhpp_viser import Viewer
        self._viewer = Viewer(self.robot)
        self._viewer.initViewer(open=open, loadModel=True)
        self._viewer.setProblem(self.problem)
        self._viewer.setGraph(self.graph)
        self._viewer(self.q_init)
        print("Viser viewer ready.  Use o.view(q) or o.play(path).")

    def view(self, q=None):
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        self._viewer(q if q is not None else self.q_init)

    def play(self, path, n=100, dt=0.05):
        """Play a path in Viser by sampling n configurations."""
        import time as _time
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        try:
            self._viewer.loadPath(path)
        except Exception:
            pass
        t0 = path.timeRange().first
        tf = path.timeRange().second
        for i in range(n):
            t = t0 + i * (tf - t0) / (n - 1)
            q = path.eval(t)[0]
            self._viewer(q)
            _time.sleep(dt)

    # ── Robot state sync ──────────────────────────────────────────────────────

    def sync_from_robot(self, timeout: float = 5.0):
        """Update q_init from the current Gazebo robot state."""
        if self._ros_node is None:
            self._ros_node = rclpy.create_node("hpp_sync_node")
            _own_node = True
        else:
            _own_node = False

        joint_state = [None]
        odom_state  = [None]

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
            if joint_state[0] is None: missing.append("/joint_states")
            if odom_state[0] is None:  missing.append("/odom")
            print(f"sync_from_robot: timeout — could not receive {', '.join(missing)}")
            return

        odom   = odom_state[0]
        bx     = odom.pose.pose.position.x
        by     = odom.pose.pose.position.y
        q_odom = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q_odom.w * q_odom.z + q_odom.x * q_odom.y)
        cosy_cosp = 1.0 - 2.0 * (q_odom.y * q_odom.y + q_odom.z * q_odom.z)
        theta  = np.arctan2(siny_cosp, cosy_cosp)

        js     = joint_state[0]
        js_map = dict(zip(js.name, js.position))

        def _read_arm(side):
            return np.array([
                js_map.get(f"tiago_pro/arm_{side}_{i}_joint",
                           js_map.get(f"arm_{side}_{i}_joint", 0.0))
                for i in range(1, 8)
            ])

        bi = self._base_idx
        ri = self._right_arm_idx
        li = self._left_arm_idx

        left_arm  = _read_arm("left")
        right_arm = _read_arm("right")

        self.q_init[bi]      = bx
        self.q_init[bi + 1]  = by
        self.q_init[bi + 2]  = np.cos(theta)
        self.q_init[bi + 3]  = np.sin(theta)
        self.q_init[ri:ri+7] = right_arm
        self.q_init[li:li+7] = left_arm

        # Rebuild locked joints with the synced left arm values so the
        # constraint graph stays consistent with q_init.
        self._left_arm_lock_values = left_arm.tolist()
        print("  Rebuilding constraint graph with synced left arm …")
        self._setup_graph()

        print(
            f"sync_from_robot: base=({bx:.3f}, {by:.3f}, θ={np.degrees(theta):.1f}°)"
            f"  right_arm={np.round(right_arm, 3).tolist()}"
            f"  left_arm={np.round(left_arm, 3).tolist()}"
        )

    # ── Combined ──────────────────────────────────────────────────────────────

    def plan_and_execute(self, max_attempts: int = 50):
        """Plan then immediately execute."""
        if self.plan(max_attempts=max_attempts):
            self.execute()
