# Demo 06 — TIAGo Pro Pick-and-Place (Whole Body MPC + HPP)

Pick-and-place demo with TIAGo Pro: the robot drives its mobile base to a table,
grasps a box with the right arm, and places it at a different position.

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

### Key files

| File | Role |
|---|---|
| `launch/bringup_simu.launch.py` | Main launch file |
| `config/agimus_controller_params.yaml` | MPC node parameters |
| `config/ocp_definition_file.yaml` | OCP definition (horizon, costs…) |
| `hpp_ws/src/tiago_pro_hpp_playground/orchestrator.py` | HPP planning + MPC publishing class |
| `hpp_ws/src/tiago_pro_hpp_playground/orchestrator_node.py` | Interactive IPython entry point |

---

## Running the demo

### Step 1 — Launch Gazebo + MPC controller

```bash
source /home/user/devel/ros2_config.sh
cd /home/user/devel/ros2_ws
colcon build --symlink-install   # only if not already built

ros2 launch agimus_demo_06_tiago_pro_pick_and_place bringup_simu.launch.py \
    use_hpp_bridge:=true
```

This starts:
- Gazebo with TIAGo Pro
- Table and box spawned at the correct positions
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
# Plan only
o.plan()

# Execute (publishes trajectory to /mpc_input)
o.execute()

# Or both at once
o.plan_and_execute()
```

---

## Running the orchestrator manually (alternative)

If xterm is unavailable or you prefer a separate terminal:

```bash
# Terminal 2 — source both environments (ros2_ws install first, then HPP takes priority)
source /home/user/devel/ros2_config.sh
source /home/user/devel/ros2_ws/install/setup.bash
source /home/user/devel/hpp_config.sh
cd /home/user/devel/hpp_ws/src/tiago_pro_hpp_playground
python3 orchestrator_node.py
```

Launch the demo without the bridge flag:

```bash
ros2 launch agimus_demo_06_tiago_pro_pick_and_place bringup_simu.launch.py
```

---

## Scene geometry

```
Table origin (base_link): x=2.0, y=0.0, z=-0.1
Table surface:             z = 0.630 m
Box (5 cm cube):           x=2.545, y=-0.15, z=0.655
Box goal position:         x=2.545, y=+0.15, z=0.655
```

The robot starts at the origin facing +x and must drive to the table.

---

## OCP costs (`config/ocp_definition_file.yaml`)

Pure trajectory tracking — HPP provides a complete joint trajectory so no
end-effector cost is needed.

| Cost | Residual | Running | Terminal |
|------|----------|---------|----------|
| `state_reg` | `ResidualModelState` | ✓ | ✓ |
| `control_reg` | `ResidualModelControl` | ✓ | — |

**Why no EE cost?**
HPP plans a complete joint trajectory `(q, v)`. The EE pose is implicitly
determined by the joint configuration via forward kinematics. Adding a
`ResidualModelFramePlacement` on top of `state_reg` is redundant and can
create conflicting objectives.

---

## Troubleshooting

**MPC waiting for buffer:** normal — it waits until the orchestrator publishes
on `/mpc_input`. Run `o.plan_and_execute()` in the orchestrator shell.

**IPython not found:** install in the HPP venv (see Prerequisites above).

**xterm not found:** install via apt or launch the orchestrator manually.
