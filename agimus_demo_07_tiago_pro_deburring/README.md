# Demo 07 — TIAGo Pro Deburring (Whole Body MPC + HPP)

Deburring demo with TIAGo Pro: the robot drives its mobile base toward a pylone
sitting on a table, approaches a hole on the right face with the right arm, inserts
the tool, then retracts.

Motion planning is handled by **HPP** (Humanoid Path Planner) and execution by
the **agimus_controller** (Whole Body MPC).

---

## Architecture

```
HPP orchestrator (xterm, HPP env)
  └─> publishes MpcInput on /mpc_input
        └─> agimus_controller (Whole Body MPC)
              └─> linear_feedback_controller (LFC)
                    └─> joint torque commands → Gazebo
```

**Active DOFs:** planar base (x, y, θ) + right arm (7) = 10

The left arm, torso, wheels, grippers, and head are locked via `LockedJoint`
constraints in the HPP model.

### Planning pipeline

```
q_init → [p1: approach] → qpg → [p2: insertion] → qg
                                                    ↓
                          qpg ← [p3: retraction] ←─┘
```

- **p1** (approach): robot drives to pre-grasp pose in front of the hole
- **p2** (insertion): right arm inserts along handle Z (world +Y)
- **p3** (retraction): reverse of p2

### Key files

| File | Role |
|---|---|
| `launch/bringup_simu.launch.py` | Main launch file |
| `config/agimus_controller_params.yaml` | MPC node parameters |
| `config/hpp_orchestrator_params.yaml` | Scene geometry, handle, tuck poses, MPC weights |
| `config/ocp_definition_file.yaml` | OCP definition (horizon, costs…) |
| `hpp/orchestrator.py` | HPP planning + MPC publishing class |
| `hpp/orchestrator_node.py` | Interactive IPython entry point |

---

## Running the demo

### Step 1 — Launch Gazebo + MPC controller

```bash
source /home/user/devel/ros2_config.sh
cd /home/user/devel/ros2_ws
colcon build --symlink-install   # only if not already built

ros2 launch agimus_demo_07_tiago_pro_deburring bringup_simu.launch.py \
    use_hpp_bridge:=true
```

This starts:
- Gazebo with TIAGo Pro
- Table and pylone spawned at the correct positions
- Linear Feedback Controller (LFC) + joint state estimator
- agimus_controller (MPC, waiting for trajectory)
- An **xterm** terminal with the HPP orchestrator (once joints are non-zero)

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `use_hpp_bridge` | `false` | Open the HPP orchestrator xterm |
| `use_mpc_debugger` | `false` | Launch MPC debugger (RViz prediction) |
| `gzclient` | `True` | Show Gazebo GUI |
| `use_gazebo` | `true` | Use Gazebo simulation |

### Step 2 — Plan and execute (in the xterm)

Once the xterm opens, an IPython shell is available with `o` (the orchestrator):

```python
# Plan approach + insertion + retraction
o.plan()

# Execute (publishes p1 + p2 + p3 to /mpc_input)
o.execute()

# Or both at once
o.plan_and_execute()

# Visualise in Viser
o.init_viewer()
o.play(o.p1)   # animate approach
o.play(o.p2)   # animate insertion
o.play(o.p3)   # animate retraction
```

---

## Running the orchestrator manually (alternative)

If xterm is unavailable or you prefer a separate terminal:

```bash
# Terminal 2 — source both environments (ros2_ws install first, then HPP takes priority)
source /home/user/devel/ros2_config.sh
source /home/user/devel/ros2_ws/install/setup.bash
source /home/user/devel/hpp_config.sh
python3 /home/user/devel/ros2_ws/install/agimus_demo_07_tiago_pro_deburring/share/agimus_demo_07_tiago_pro_deburring/hpp/orchestrator_node.py
```

Launch the demo without the bridge flag:

```bash
ros2 launch agimus_demo_07_tiago_pro_deburring bringup_simu.launch.py
```

---

## Scene geometry

All positions are defined in `config/hpp_orchestrator_params.yaml` (single source of truth
for both the HPP orchestrator and the Gazebo launch file).

```
Table origin (base_link): x=1.5, y=0.0, z=0.0
Table surface:             z = 0.73 m
Pylone base:               x=1.5, y=0.0, z=1.03 m
Hole (hole_tiago_25):      face y=-0.213, x=0.0, z=0.0 (pylone frame)
```

The robot starts at the origin and must drive to the pylone facing the right face (−Y side).

---

## Handle definition

The deburring handle is added programmatically in `orchestrator.py` (not from the SRDF):

```python
# Rx(-90°): handle Z = world +Y = into the hole
R = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
robot.addHandle("pylone/pylone_link", "pylone/hole_tiago_25", SE3(R, T), 0.05, 6*[True])
robot.handles()["pylone/hole_tiago_25"].approachingDirection = [0, 0, 1]
```

The `approachingDirection = [0, 0, 1]` means the gripper moves along +Z of the handle
frame (= world +Y = into the hole) during the approach transition.

---

## OCP costs (`config/ocp_definition_file.yaml`)

Pure trajectory tracking — HPP provides a complete joint trajectory so no
end-effector cost is needed.

| Cost | Residual | Running | Terminal |
|------|----------|---------|----------|
| `state_reg` | `ResidualModelState` | ✓ | ✓ |
| `control_reg` | `ResidualModelControl` | ✓ | — |

---

