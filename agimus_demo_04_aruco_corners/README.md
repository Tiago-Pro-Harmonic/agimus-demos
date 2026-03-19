AGIMUS demo 04 ArUco corners — TIAGo Pro
-----------------------------------------

> [!CAUTION]
> This demo is **CPU intensive** and strongly relies on short MPC computation times. Make sure you **run this demo on a capable computer**. On the real robot, always keep the emergency stop within reach and ensure the workspace is clear before advancing to the next corner.

This demo uses the [agimus_controller](https://github.com/agimus-project/agimus_controller) (MPC with Crocoddyl) to drive the TIAGo Pro's right gripper fingertip (`gripper_right_fingertip_left_link`) to hover in front of each of the **4 corners** of a fixed ArUco marker (DICT_4X4_50, 17.6 cm) in sequence. The user advances from corner to corner manually via a ROS 2 service call, giving full control of pacing.

**In simulation** the marker pose is fixed via a static TF (`base_footprint → aruco_marker`) set through launch arguments. A 3D model of the marker is also spawned in Gazebo.

**On the real robot** the marker can be detected live by the TIAGo Pro head camera using the `ros2_aruco` package, or fixed via a static TF if `use_aruco_detection:=false`. When detection is enabled, the `aruco_corner_publisher` node subscribes to `/aruco_markers` and broadcasts the `aruco_marker` TF dynamically, latching the last known pose when the marker temporarily goes out of view.

**Corner ordering** follows the OpenCV ArUco convention in the marker frame (X right, Y up, Z pointing out toward the robot):

```
Corner 0  top-left:     (-h,  h, z)
Corner 1  top-right:    ( h,  h, z)
Corner 2  bottom-right: ( h, -h, z)
Corner 3  bottom-left:  (-h, -h, z)
```
where `h = marker_size / 2 = 0.088 m` and `z = approach_z_offset` (default `0.02 m`).

After all 4 corners are visited the robot returns to its neutral arm configuration and holds there.

---

### Dependencies

This demo requires source-built dependencies found in:
- [control.repos](../control.repos)

---

### Build

```bash
cd ros2_ws
source ../ros2_config.sh
colcon build --symlink-install --packages-select agimus_demo_04_aruco_corners
source install/setup.bash
```

---

### Simulation

> [!NOTE]
> Gazebo simulation of TIAGo Pro requires a high-frequency simulated environment. Users may experience high CPU utilization on less powerful machines.

Launch with the default marker pose (0.6 m forward, 1.2 m up, facing the robot):

```bash
ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py
```

To disable the Gazebo GUI (headless mode):

```bash
ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py gzclient:=False
```

The marker pose can be customised through launch arguments:

```bash
ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py \
  marker_x:=0.6 marker_y:=0.0 marker_z:=1.2
```

**Expected startup sequence:**
1. Gazebo opens with the TIAGo Pro in its initial configuration.
2. The `wait_for_non_zero_joints` node confirms the robot is ready.
3. The MPC controller initialises and logs: `MPC is initialized and buffer has enough data`.
4. The `aruco_corner_publisher` logs: `ArucoCornerPublisher ready. Will visit 4 corners …`
5. The robot holds still at **corner 0** — it will not move until you issue a service call.

---

### Real robot

> [!WARNING]
> Always keep the emergency stop within reach. Ensure the workspace around the marker is clear of obstacles before advancing to each corner. The demo has **no collision avoidance**.

> [!CAUTION]
> Before starting, move the robot to a safe position with sufficient joint motion range. Ensure all spectators are at a safe distance.

#### Docker requirement

> [!IMPORTANT]
> The real robot runs **ROS Humble**. This repository uses **ROS Jazzy**, which is incompatible and will crash the robot. You must use the **Agimus Docker** (ROS Humble) to communicate with the real robot. Do **not** use the Jazzy Docker from this repository for real robot operation.

#### Network setup (CycloneDDS)

Communication with the TIAGo Pro uses CycloneDDS. The following environment variables must be set in the Docker (already configured in `ros2_config.sh`):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/tmp/cyclone_config.xml
export ROS_DOMAIN_ID=2
```

Copy the CycloneDDS config to `/tmp/` inside the Docker at each startup:

```bash
cp agimus-demos/agimus_demos_common/config/tiago_pro/* /tmp/
scp agimus-demos/agimus_demos_common/config/tiago_pro/* pal@tiago-pro:/tmp/
```

Edit `cyclone_config.xml` to set the correct **network interface** for your machine (replace `enp0s31f6` with the interface connected to the robot network).

#### Launch the demo

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py
```

By default, live ArUco detection from the head camera is enabled (`use_aruco_detection:=true`). To use a fixed static TF instead:

```bash
ros2 launch agimus_demo_04_aruco_corners bringup.launch.py \
  use_aruco_detection:=false \
  marker_x:=0.6 marker_y:=0.0 marker_z:=1.2
```

**Expected startup sequence:**

1. The LFC controller activates on the robot.
2. `wait_for_non_zero_joints` confirms the robot is ready.
3. The MPC controller and `aruco_corner_publisher` start.
4. If detection is enabled, the `aruco_corner_publisher` logs:
   ```
   ArUco detection enabled — subscribing to /aruco_markers for marker ID 0.
   ArucoCornerPublisher ready. Will visit 4 corners …
   ```
5. The robot holds still — it will not move until you issue a service call.

> [!IMPORTANT]
> Before calling `next_corner`, confirm the marker is being detected (`ros2 topic echo /aruco_markers`) and that the predicted EE target in RViz is in front of the expected corner.

**Verify ArUco detection** (optional sanity check before launching):

```bash
ros2 run ros2_aruco aruco_node --ros-args \
  -p marker_size:=0.176 \
  -p aruco_dictionary_id:=DICT_4X4_50 \
  -p image_topic:=/head_front_camera/image \
  -p camera_info_topic:=/head_front_camera/camera_info
```

Then in another terminal:

```bash
ros2 topic echo /aruco_markers
```

You should see messages with `marker_ids: [0]` and a valid pose. Kill this node before launching the demo.

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
> `config/trajectory_weights_params.yaml` to enable **automatic** advancement.
> The default `dwell_time: 0.0` disables auto-advance entirely.

---

### Optional: MPC debugger (trajectory visualisation in RViz)

```bash
ros2 launch agimus_demo_04_aruco_corners bringup_simu.launch.py use_mpc_debugger:=true
```

This launches `mpc_debugger_node`, which publishes the MPC's predicted EE trajectory as a path marker in RViz. Useful for verifying that the OCP is converging toward the correct corner target.

---

### Tuning the controller

All tunable parameters are in the `config/` directory.

#### `config/trajectory_weights_params.yaml` — reference publisher

| Parameter | Default | Effect |
|-----------|---------|--------|
| `w_pose` | `[20, 20, 20, 0, 0, 0]` | EE position tracking weight `[x, y, z, rx, ry, rz]`. Orientation weights are 0: only position is tracked. Increase to track corners more precisely; too high can cause oscillation. |
| `w_q` | `[2.0]` | Joint position regularisation weight (broadcast to all 7 DOF). Increase to keep the arm closer to the reference configuration. |
| `w_qdot` | `[8.0]` | Joint velocity regularisation weight. Increase to reduce speed. |
| `w_robot_effort` | `[0.0012]` | Torque regularisation weight. Very small; mainly keeps the OCP well-conditioned. |
| `approach_z_offset` | `0.02` | Distance (m) the EE hovers in front of the marker plane (along marker Z axis, toward the robot). Increase for a safer standoff on the real robot. |
| `marker_size` | `0.176` | Physical size of the ArUco marker in metres. Must match the real marker. |
| `dwell_time` | `0.0` | Auto-advance interval in seconds. `0.0` = manual only (service call required). |

#### `config/ocp_definition_file.yaml` — OCP cost structure

| Cost | Default weight | Effect |
|------|---------------|--------|
| `state_reg` | `1.0` | Penalises deviation from reference joint configuration and zero velocity. |
| `control_reg` | `1.0` | Penalises joint torques. |
| `goal_tracking` | `10.0` | Weight of `ResidualModelVisualServoing`. Multiplied by `w_pose` values above. Increase to make the EE converge faster; decrease if motion is too aggressive. |

**Tuning strategy:**
- If the EE does not reach the corner accurately: increase `goal_tracking` and/or `w_pose`.
- If the robot moves too fast or oscillates: reduce `goal_tracking` and/or increase `w_qdot`.
- If the arm drifts far from a natural posture: increase `w_q`.

#### `config/agimus_controller_params.yaml` — MPC solver

| Parameter | Default | Effect |
|-----------|---------|--------|
| `ocp.horizon_size` | `19` | Number of OCP nodes. Increase for longer look-ahead (better for large motions, more CPU). |
| `ocp.dt` | `0.01` | Base timestep in seconds. |
| `ocp.dt_factor_n_seq` | `[1,2,4,8] × [5,5,5,4]` | Multi-rate horizon: the first 5 nodes use `dt`, the next 5 use `2×dt`, etc. |
| `ocp.max_iter` | `5` | Maximum solver iterations per cycle. Increase for better convergence at the cost of computation time. |
| `ocp.max_qp_iter` | `50` | Maximum inner QP iterations. |

---

### Architecture overview

**Simulation** (`bringup_simu.launch.py`, `use_aruco_detection:=false`):

```
static_transform_publisher
(base_footprint → aruco_marker, fixed)
        │ TF
        ▼
aruco_corner_publisher        ~/next_corner service (Trigger)
  publishes /mpc_input  ──────────────────────────────────┘
        │
        ▼
agimus_controller_node (MPC 50 Hz)
  ResidualModelVisualServoing:
    wMtarget = TF(base_footprint → aruco_marker) × oMcorner
        │
        ▼
linear_feedback_controller → TIAGo Pro joint torques
```

**Real robot** (`bringup.launch.py`, `use_aruco_detection:=true`):

```
TIAGo Pro head camera
        │ /head_front_camera/image
        ▼
aruco_node (ros2_aruco)
        │ /aruco_markers (poses in head_front_camera_optical_frame)
        ▼
aruco_corner_publisher        ~/next_corner service (Trigger)
  broadcasts TF: aruco_marker
  publishes /mpc_input  ──────────────────────────────────┘
        │
        ▼
agimus_controller_node (MPC 50 Hz)
  ResidualModelVisualServoing:
    wMtarget = TF(base_footprint → aruco_marker) × oMcorner
        │
        ▼
linear_feedback_controller → TIAGo Pro joint torques
```

**Key files:**

| File | Purpose |
|------|---------|
| `agimus_demo_04_aruco_corners/aruco_corner_publisher.py` | State machine + MPC reference publisher |
| `config/ocp_definition_file.yaml` | OCP cost structure (Crocoddyl) |
| `config/agimus_controller_params.yaml` | MPC solver parameters |
| `config/trajectory_weights_params.yaml` | EE and joint weights, geometry parameters |
| `launch/bringup_simu.launch.py` | Simulation launch (Gazebo + TIAGo Pro) |
| `launch/bringup.launch.py` | Real robot launch |

---

### Known limitations / implementation notes

- **Collision avoidance is disabled.** The OCP has no collision cost. Ensure the workspace is clear before running on the real robot.
- **Dummy collision pair required.** The `agimus_controller` parameter schema has a bug where `collision_pairs_names` defaults to `[""]`, causing a crash if no valid pair is provided. A dummy pair (`arm_right_7_link_0`, `arm_left_7_link_0`) is declared in `agimus_controller_params.yaml` as a workaround. It has no effect on the OCP.
- **Orientation is not controlled.** `w_pose` orientation weights are `[0, 0, 0]`. The EE orientation is regulated only by the joint-space state cost. This is intentional for corner pinpointing (position only).
- **Marker latching on real robot.** When `use_aruco_detection:=true`, the last detected marker pose is latched and re-broadcast at 100 Hz. If the marker has never been seen, the MPC has no valid TF and will stall. Ensure the marker is visible to the camera before the `aruco_corner_publisher` initialises.
- **Single marker ID tracked.** Only the marker matching `target_marker_id` (default 0) is used. All others are ignored.
