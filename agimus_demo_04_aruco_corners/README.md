AGIMUS demo 04 ArUco corners
----------------------------

> [!CAUTION]
> This demo is **CPU intensive** and strongly relies on short MPC computation times. Make sure you **run this demo on a capable computer**. On the real robot, always keep the emergency stop within reach and ensure the workspace is clear before advancing to the next corner.

This demo uses the [agimus_controller](https://github.com/agimus-project/agimus_controller) (MPC with Crocoddyl) to drive the Franka Panda's `fer_hand_tcp` frame to hover **2 cm above each of the 4 corners** of a fixed ArUco marker (DICT_4X4_50, 17.6 cm) in sequence. The user advances from corner to corner manually via a ROS 2 service call, giving full control of pacing.

**In simulation** the marker pose is fixed via a static TF (` → aruco_marker`) set through launch arguments.

**On the real robot** the marker is detected live by a wrist-mounted RealSense D435 camera using the `ros2_aruco` package. The `aruco_corner_publisher` node subscribes to the detections and broadcasts the `camera_color_optical_frame → aruco_marker` TF dynamically, latching the last known pose when the marker temporarily goes out of view.

**Corner ordering** follows the OpenCV ArUco convention in the marker frame (X right, Y up, Z pointing out toward the robot):

```
Corner 0  top-left:     (-h,  h, z)
Corner 1  top-right:    ( h,  h, z)
Corner 2  bottom-right: ( h, -h, z)
Corner 3  bottom-left:  (-h, -h, z)
```
where `h = marker_size / 2 = 0.088 m` and `z = approach_z_offset = 0.02 m`.

After all 4 corners are visited the robot returns to the Franka neutral configuration and holds there.

---

### Dependencies

This demo requires source-built dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)

---

### Build

```bash
colcon build --packages-select agimus_demo_04_aruco_corners
source install/setup.bash
```

---

### Simulation

> [!NOTE]
> Gazebo simulation of Franka robots requires a high-frequency simulated environment. Users may experience high CPU utilization on less powerful machines.

Launch with default marker pose (0.5 m forward, 0.3 m up, facing the robot):

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py use_gazebo:=true use_rviz:=true
```

The marker pose can be customised through launch arguments:

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \
  use_gazebo:=true use_rviz:=true \
  marker_x:=0.5 marker_y:=0.0 marker_z:=0.3 \
  marker_roll:=0.0 marker_pitch:=3.14159 marker_yaw:=0.0
```

All pose values are in the `` frame. The default `marker_pitch:=π` rotates the marker so its Z axis points toward the robot (i.e. the marker faces the robot).

**Expected startup sequence:**
1. Gazebo and RViz open, the Franka arm appears in its home configuration.
2. The `wait_for_non_zero_joints` node confirms the robot is ready.
3. The MPC controller initialises (~2 s) and logs: `MPC is initialized and buffer has enough data`.
4. The `aruco_corner_publisher` logs: `ArucoCornerPublisher ready. Will visit 4 corners …`
5. The robot holds still at **corner 0** — it will not move until you issue a service call.

---

### Controlling the demo — advancing between corners

The robot stays at each corner until you explicitly trigger the next one:

```bash
ros2 service call /aruco_corner_publisher/next_corner std_srvs/srv/Trigger
```

Each call advances one step in the sequence:
`CORNER_0 → CORNER_1 → CORNER_2 → CORNER_3 → NEUTRAL → (done, robot holds at neutral)`

The service response reports the transition:
```
success: True
message: Advanced from CORNER_0 to CORNER_1.
```

Calling the service when the sequence is complete returns `success: False`.

> [!TIP]
> Set `dwell_time` to a positive number of seconds in
> `config/trajectory_weights_params.yaml` to re-enable **automatic** advancement
> (useful for unattended simulation runs). The default `dwell_time: 0.0` disables
> auto-advance entirely.

---

### Real robot

> [!WARNING]
> Always keep the emergency stop within reach. Ensure the workspace around the marker is clear of obstacles before advancing to each corner. The demo has **no collision avoidance**.

> [!CAUTION]
> Turn on the robot and unlock its joints in the Franka web UI. Place the robot in a safe initial configuration (arm roughly vertical, joints near neutral). The ArUco marker must be placed in a pose that is reachable by the end-effector without risk of collision with the table or surroundings.

**Step 1 — Start the RealSense D435 camera driver** (in a dedicated terminal):

```bash
ros2 launch realsense2_camera rs_launch.py
```

Verify the camera is publishing by checking that these topics are active:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/camera_info
```

**Step 2 — Verify ArUco detection** (optional sanity check, requires the demo to be built):

You can run the ArUco node standalone to confirm the marker is visible before starting the demo:

```bash
ros2 run ros2_aruco aruco_node --ros-args \
  -p marker_size:=0.176 \
  -p aruco_dictionary_id:=DICT_4X4_50 \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info
```

Then in another terminal:

```bash
ros2 topic echo /aruco_markers
```

You should see messages with `marker_ids: [0]` and a pose. Kill this node before launching the demo (it will be re-launched automatically).

**Step 3 — Launch the demo** (in a new terminal):

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \
  robot_ip:=<robot-ip> \
  use_aruco_detection:=true \
  use_rviz:=true
```

If the robot controller runs on a separate real-time computer, add:

```bash
  aux_computer_ip:=<aux-ip> \
  aux_computer_user:=<username>
```

**Expected startup sequence:**

1. The Franka arm initialises and the linear feedback controller activates.
2. `wait_for_non_zero_joints` confirms the robot is ready.
3. The MPC controller and `aruco_corner_publisher` start.
4. The `aruco_corner_publisher` logs:
   ```
   [aruco_corner_publisher]: ArUco detection enabled — subscribing to /aruco_markers for marker ID 0.
   [aruco_corner_publisher]: ArucoCornerPublisher ready. Will visit 4 corners …
   ```
5. The robot holds still — it will not move until you issue a service call.

> [!IMPORTANT]
> Before calling `next_corner`, confirm the marker is being detected (`ros2 topic echo /aruco_markers`) and that the predicted EE target in RViz is above the expected corner. The robot will move as soon as you call the service.

**If using a different marker ID** (default is 0), edit `config/trajectory_weights_params.yaml` before building:

```yaml
target_marker_id: <ID>
```

Then rebuild:

```bash
colcon build --packages-select agimus_demo_04_aruco_corners && source install/setup.bash
```

---

### Optional: MPC debugger (trajectory visualisation in RViz)

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \
  use_gazebo:=true use_rviz:=true use_mpc_debugger:=true
```

This launches `mpc_debugger_node`, which publishes the MPC's predicted EE trajectory as a path marker in RViz. Useful for verifying that the OCP is converging toward the correct corner target.

---

### Tuning the controller

All tunable parameters are in the `config/` directory.

#### `config/trajectory_weights_params.yaml` — reference publisher

| Parameter | Default | Effect |
|-----------|---------|--------|
| `w_pose` | `[20, 20, 20, 0, 0, 0]` | EE position tracking weight `[x, y, z, rx, ry, rz]`. Orientation weights are 0: only position is tracked, orientation is regulated by joint-space cost. Increase to track corners more precisely; too high can cause oscillation. |
| `w_q` | `[2.0]` | Joint position regularisation weight (broadcast to all 7 DOF). Increase to keep the arm closer to the reference configuration. |
| `w_qdot` | `[8.0]` | Joint velocity regularisation weight. Increase to reduce speed. |
| `w_robot_effort` | `[0.0012]` | Torque regularisation weight. Very small; mainly keeps the OCP well-conditioned. |
| `approach_z_offset` | `0.02` | Distance (m) the EE hovers in front of the marker plane. Increase for a safer standoff on the real robot. |
| `marker_size` | `0.176` | Physical size of the ArUco marker in metres. Must match the real marker. |
| `dwell_time` | `0.0` | Auto-advance interval in seconds. `0.0` = manual only (service call required). |

#### `config/ocp_definition_file.yaml` — OCP cost structure

| Cost | Default weight | Effect |
|------|---------------|--------|
| `state_reg` | `1.0` | Penalises deviation from the reference joint configuration and zero velocity. Balances against `goal_tracking`. |
| `control_reg` | `1.0` | Penalises joint torques. |
| `goal_tracking` | `10.0` | Weight of `ResidualModelVisualServoing`. This is multiplied by the per-axis `w_pose` values above. Increase to make the EE converge faster; decrease if motion is too aggressive. |

**Tuning strategy:**
- If the EE does not reach the corner accurately: increase `goal_tracking` and/or `w_pose`.
- If the robot moves too fast or oscillates: reduce `goal_tracking` and/or increase `w_qdot`.
- If the arm drifts far from a natural posture during EE tracking: increase `w_q`.

#### `config/agimus_controller_params.yaml` — MPC solver

| Parameter | Default | Effect |
|-----------|---------|--------|
| `ocp.horizon_size` | `19` | Number of OCP nodes. Increase for longer look-ahead (better for large motions, more CPU). |
| `ocp.dt` | `0.01` | Base timestep in seconds. |
| `ocp.dt_factor_n_seq` | `[1,2,4,8] × [5,5,5,4]` | Multi-rate horizon: the first 5 nodes use `dt`, the next 5 use `2×dt`, etc. |
| `ocp.max_iter` | `10` | Maximum solver iterations per cycle. Increase for better convergence at the cost of computation time. |
| `ocp.max_qp_iter` | `100` | Maximum inner QP iterations. |

---

### Architecture overview

**Simulation** (`use_aruco_detection:=false`):

```
static_transform_publisher
( → aruco_marker, fixed)
        │ TF
        ▼
aruco_corner_publisher        ~/next_corner service (Trigger)
  publishes /mpc_input ───────────────────────────────────┘
        │
        ▼
agimus_controller_node (MPC 100 Hz)
  ResidualModelVisualServoing:
    wMtarget = TF(→aruco_marker) × oMcorner
        │
        ▼
linear_feedback_controller → Franka joint torques
```

**Real robot** (`use_aruco_detection:=true`):

```
RealSense D435 (wrist camera)
        │ /camera/camera/color/image_raw
        ▼
aruco_node (ros2_aruco)
        │ /aruco_markers (poses in camera_color_optical_frame)
        ▼
aruco_corner_publisher        ~/next_corner service (Trigger)
  broadcasts TF: camera_color_optical_frame → aruco_marker
  publishes /mpc_input ───────────────────────────────────┘
        │
        ▼
agimus_controller_node (MPC 100 Hz)
  ResidualModelVisualServoing:
    wMtarget = TF(→aruco_marker) × oMcorner
              [resolved through full kinematic chain]
        │
        ▼
linear_feedback_controller → Franka joint torques
```

**Key files:**

| File | Purpose |
|------|---------|
| `agimus_demo_04_aruco_corners/aruco_corner_publisher.py` | State machine + MPC reference publisher |
| `config/ocp_definition_file.yaml` | OCP cost structure (Crocoddyl) |
| `config/agimus_controller_params.yaml` | MPC solver parameters |
| `config/trajectory_weights_params.yaml` | EE and joint weights, geometry parameters |
| `launch/bringup.launch.py` | Full system launch (simulation or real robot) |

---

### Known limitations / implementation notes

- **Collision avoidance is disabled.** The OCP has no collision cost. Ensure the workspace is clear before running on the real robot.
- **Dummy collision pair required.** The `agimus_controller` parameter schema has a bug where `collision_pairs_names` defaults to `[""]`, causing a crash if no valid pair is provided. A dummy robot-only pair (`fer_hand_sc_capsule_0`, `fer_link5_sc_capsule_0`) is declared in `agimus_controller_params.yaml` as a workaround. It has no effect on the OCP since there is no collision cost term.
- **Orientation is not controlled.** `w_pose` orientation weights are `[0, 0, 0]`. The EE orientation is regulated only by the joint-space state cost, which pulls toward the reference configuration. This is intentional for corner pinpointing (position only), but means the EE approach direction may vary.
- **MPC uses `fer_hand_tcp`, not a fingertip frame.** The hover point is 2 cm in front of the marker plane at the `fer_hand_tcp` origin. Adjust `approach_z_offset` and/or the frame name if a different contact point is needed.
- **Marker latching on real robot.** When `use_aruco_detection:=true`, the last detected marker pose is latched and re-broadcast at 100 Hz. If the marker has never been seen, the MPC has no valid TF and will stall. Ensure the marker is visible to the camera before the `aruco_corner_publisher` initialises.
- **Camera calibration matters.** The accuracy of corner pinpointing on the real robot depends entirely on the quality of the hand-eye calibration between `fer_hand_tcp` and `camera_color_optical_frame`. Poor calibration will shift all 4 corners by the same offset.
- **Single marker ID tracked.** Only the marker matching `target_marker_id` (default 0) is used. If multiple markers are present in the scene, all others are ignored.
