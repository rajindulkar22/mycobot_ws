# mycobot_moveit_projects

C++ MoveIt 2 learning nodes for the myCobot 280 JN. These nodes plan and execute arm motions through the same `arm_controller` action server used by the Python sim nodes and RViz.

**Prerequisites:** Gazebo running, then `move_group` (see [workspace README](../README.md#moveit-2)).

---

## Nodes

| Executable | Launch file | Purpose |
|------------|-------------|---------|
| `named_targets` | `named_targets.launch.py` | Plan/execute SRDF named states (`home` → `grasp_approach` → `home`) |
| `pose_target` | `pose_target.launch.py` | FK read TCP → Z−3 cm → IK → OMPL → execute |
| `cartesian_path` | `cartesian_path.launch.py` | MoveIt Cartesian path: `grasp_approach` → straight Z−3 cm → return → `home` |
| `cube_approach` | `cube_approach.launch.py` | MoveIt pick-and-place: approach → pre-grasp → close → lift → place → release |
| `planning_scene_objects` | *(run directly)* | Add table + 25 mm cube boxes to MoveIt planning scene |
| `test_obstacle` | *(run directly)* | Add/remove a 10 cm box at the grasp_approach TCP pose |

---

## How to run

**Recommended — combined stack (keeps sim `/clock` and TF in sync):**

```bash
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py
```

Wait ~15 s for Gazebo + controllers + move_group. Or use the Sim Workbench **Start Gazebo + MoveIt** button.

**Separate terminals (restart move_group whenever Gazebo restarts):**

```bash
# Terminal 1 — Gazebo + controllers
ros2 launch mycobot_280jn_sim gazebo_sim.launch.py

# Terminal 2 — move_group (after Gazebo is up)
ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py

# Terminal 3 — pick a demo
ros2 launch mycobot_moveit_projects named_targets.launch.py
# or
ros2 launch mycobot_moveit_projects pose_target.launch.py
# or
ros2 launch mycobot_moveit_projects cartesian_path.launch.py
# or
ros2 launch mycobot_moveit_projects cube_approach.launch.py
```

**Planning scene sync (optional, before planning in RViz or C++ nodes):**

```bash
ros2 run mycobot_moveit_projects planning_scene_objects
```

**Test obstacle (blocks path at grasp pose):**

```bash
# Add
ros2 run mycobot_moveit_projects test_obstacle --ros-args -p operation:=add

# Remove
ros2 run mycobot_moveit_projects test_obstacle --ros-args -p operation:=remove
```

**Build:**

```bash
colcon build --packages-select mycobot_moveit_projects
source install/setup.bash
```

Launch files inject MoveIt config (URDF, SRDF, kinematics, OMPL, joint limits) and set `use_sim_time: true`.

---

## Named targets — process

Named targets skip explicit FK/IK: MoveIt loads joint values from the SRDF (`home`, `grasp_approach`, etc.), plans with OMPL, and executes.

```
SRDF named state (joint angles)
        ↓  OMPL planning
Collision-free joint trajectory
        ↓  arm_controller
Robot moves
```

Implementation: [`src/named_targets.cpp`](src/named_targets.cpp).

---

## Pose target — complete process

The `pose_target` node demonstrates Cartesian motion planning: read where the gripper is, offset the target pose, solve IK, and let OMPL find a collision-free path.

```
Current joint angles
        ↓  Forward kinematics
Current TCP pose
        ↓  subtract 3 cm from Z
Target TCP pose
        ↓  Inverse kinematics
Target joint angles
        ↓  OMPL planning
Collision-free joint trajectory
        ↓  arm_controller
Robot moves
```

### Full sequence

```
grasp_approach
    → (FK) read TCP pose
    → (offset) Z − 3 cm
    → (IK + OMPL + execute) move downward
    → grasp_approach
    → home
```

Planning uses 10% velocity/acceleration scaling, 10 s planning time, and up to 10 attempts. Goal tolerances: 5 mm position, ~1° orientation.

Implementation: [`src/pose_target.cpp`](src/pose_target.cpp).

---

## cartesian_path

MoveIt **Cartesian interpolation** (`computeCartesianPath`): move to `grasp_approach`, read TCP pose via FK, descend 3 cm in a straight line, return to the original TCP pose, then go `home`.

### Prerequisites

| Required | Optional |
|----------|----------|
| Gazebo (`gazebo_sim.launch.py`) — bridges `/clock` via `ros_gz_bridge` and publishes `/joint_states` | RViz (`gazebo_moveit_rviz.launch.py`) — visualization only |
| `move_group` (`gazebo_move_group.launch.py`) — plans and executes | |

**Do not launch `cartesian_path` before Gazebo.** The launch file sets `use_sim_time: true`. Gazebo sim time lives on the Gazebo side until `gazebo_sim.launch.py` bridges it to ROS `/clock`. Without that bridge (or if Gazebo is not running), the node waits 10 s and exits with:

```text
Exception: No simulation clock received.
```

RViz and `move_group` consume sim time; they do **not** publish it.

### Launch order (4 terminals)

Source ROS and the workspace in every terminal:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
```

| Terminal | Command |
|----------|---------|
| **1** | `ros2 launch mycobot_280jn_sim gazebo_sim.launch.py` |
| **2** | `ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py` |
| **3** *(optional)* | `ros2 launch mycobot_280jn_moveit_config gazebo_moveit_rviz.launch.py` |
| **4** | `ros2 launch mycobot_moveit_projects cartesian_path.launch.py` |

Wait until Terminal 1 shows Gazebo running and controllers active before starting Terminal 4.

### Pre-flight checks

```bash
ros2 topic hz /clock                   # must show data (~1000 Hz) after Gazebo starts
ros2 topic echo /clock --once          # must show a Clock message
ros2 topic echo /joint_states --once   # must show joint positions
ros2 node list | grep move_group       # move_group must be listed
```

Expected success log from `cartesian_path`: `Simulation clock active: ...` before any motion.

### Sequence

```
grasp_approach (named target)
    → (FK) read TCP pose
    → (Cartesian) straight Z − 3 cm
    → (Cartesian) return to original TCP
    → home (named target)
```

The node initializes the joint-state monitor **before** the first motion so the Cartesian trajectory starts from the robot's actual configuration (not zeros).

Diagnostic log lines to watch:

- `Cartesian start joint2_to_joint1: ~0.330000` — start state matches `grasp_approach`
- `Trajectory start joint2_to_joint1: ~0.330000` — trajectory first point matches start state

If start state is correct but trajectory still starts at zero, check `computeCartesianPath` / `setStartState` in the MoveIt logs.

Implementation: [`src/cartesian_path.cpp`](src/cartesian_path.cpp).

---

## cube_approach (pick-and-place)

MoveIt **approach + Cartesian pre-grasp + gripper pick-and-place** in one node. Arm motions use MoveIt; gripper uses `/gripper_action_controller/gripper_cmd` (same as `pick_cube`).

### Sequence

```
approach (IK) → grasp_hover → grasp_descend (joint-space, same as pick_cube)
     → close gripper → hold 1.5 s
     → lift grasp_hover → lift grasp_approach
     → plan to place (carry height)
     → place descend → open gripper → hold 0.5 s
     → retract to carry height → return home
```

Grasp is **physics-only in Gazebo** (friction between fingers and cube). MoveIt does not attach the object. Finger and cube contact friction are tuned in `mycobot_280jn_sim.urdf.xacro` and `mycobot_table.sdf`; restart Gazebo after changing them.

### Prerequisites

| Required | Notes |
|----------|-------|
| Gazebo + move_group | Same as other MoveIt C++ nodes |
| Gripper open at start | `ros2 run mycobot_sim_projects gripper_commander -- open` if needed |
| No `test_obstacle` | Remove blocking obstacle before run |
| SRDF changes | Restart `gazebo_move_group.launch.py` after editing `mycobot_280jn_sim.srdf` |
| Gazebo model/world changes | Restart Gazebo after URDF or world tuning |

### Place parameters (launch arguments)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `place_x` | `0.10` | Place approach TCP X in `world` |
| `place_y` | `0.15` | Place approach TCP Y in `world` |
| `place_z` | `0.140` | Carry height for approach and retract (m) |
| `place_descend_z` | `0.055` | Release height before opening gripper (m) |
| `return_home` | `true` | Plan to SRDF `home` after release |

Override at launch (ROS 2 launch arguments — not `--ros-args`):

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  place_x:=0.12 place_y:=0.18 place_z:=0.140 place_descend_z:=0.055
```

Disable return home:

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py return_home:=false
```

### Run (terminal)

```bash
# Terminals 1–2: Gazebo + move_group (as above)

ros2 launch mycobot_moveit_projects cube_approach.launch.py
```

### Run (GUI — Sim Workbench)

One window for the full stack instead of multiple terminals:

```bash
ros2 run mycobot_sim_projects sim_workbench_ui
```

1. **Simulation stack** — **Start Gazebo + MoveIt (recommended)**, or start **Gazebo** then **move_group** separately (optional **RViz**).
2. **Prepare arm** — **Open gripper**, then **Go home** (wait ~6 s after home before pick).
3. **Place target** — Set X/Y/Z and **Release Z**, then **Run cube_approach**.

Optional: `--workspace ~/mycobot_ws` if the colcon workspace is not in the default location.

For the most tuned single-command pick on the 25 mm cube, use `gazebo_pose_commander -- pick_cube` instead.

Implementation: [`src/cube_approach.cpp`](src/cube_approach.cpp).

---

## Troubleshooting (sim time / TF)

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `Detected jump back in time` (TF) | Gazebo restarted while `move_group` still running, or stale `gzserver` | Stop everything, then clean up orphans and restart together |
| MoveIt execute aborted / unknown goal response | Same — sim `/clock` jumped backward, TF buffer cleared | Same |
| Pick fails right after restarting Gazebo only | `move_group` still bound to old sim time | Never leave `move_group` running across a Gazebo restart |

**Clean shutdown before restart:**

```bash
pkill -f move_group || true
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
```

Then launch the combined stack (recommended):

```bash
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py
```

Or restart **both** Gazebo and `move_group` in separate terminals. Sim Workbench **Stop sim stack** runs the same cleanup.

**Before pick:** open gripper → go home → wait ~6 s. Avoid running `gazebo_pose_commander -- home` immediately before `cube_approach` without a pause — both use `arm_controller`.

| Grasp slips / cube drops on lift | Low finger–cube friction, fast descend, or stale Gazebo model | Rebuild `mycobot_280jn_sim`, full `pkill` Gazebo restart, use combined stack. Confirm 25 mm cube (`grep "0.025"` in installed world). Compare with `gazebo_pose_commander -- pick_cube` — if that works but MoveIt does not, restart `move_group` after SRDF changes. |

---

## planning_scene_objects

Adds two collision boxes to the MoveIt planning scene in the **world** frame (does not move Gazebo physics objects — only MoveIt's internal collision model):

| ID | Centre (world) | Size (m) |
|----|----------------|----------|
| `work_table` | (0, 0, −0.03) | 0.8 × 0.6 × 0.05 |
| `pick_cube` | (0.20, 0, 0.4125) | 0.025 cube (matches Gazebo `mycobot_table.sdf`) |

Matches the 25 mm cube placement documented in [PICK_CUBE_HANDOFF.md](../mycobot_sim_projects/PICK_CUBE_HANDOFF.md).

Implementation: [`src/planning_scene_objects.cpp`](src/planning_scene_objects.cpp).

---

## test_obstacle

Adds or removes a **10 cm cube** at `(0.101, −0.065, 0.437)` in the world frame — near the `grasp_approach` TCP pose — to demonstrate planning failure when an obstacle blocks the path.

Parameter: `operation` = `add` (default) or `remove`.

Implementation: [`src/test_obstacle.cpp`](src/test_obstacle.cpp).

---

## Key parameters (pose_target)

| Setting | Value | Role |
|---------|-------|------|
| Planning group | `arm` | Six-DOF arm (gripper excluded) |
| Pose reference frame | `world` | Cartesian targets in world |
| TCP link | `gripper_tcp` | End-effector frame for FK/IK |
| Z offset | −0.03 m | Downward motion after approach |

---

## Related docs

| Document | Contents |
|----------|----------|
| [../README.md](../README.md) | Workspace hub, all workflows |
| [../mycobot_sim_projects/THEORY.md](../mycobot_sim_projects/THEORY.md) | Control theory, MoveIt background |
| [../mycobot_280jn_moveit_config/config/mycobot_280jn_sim.srdf](../mycobot_280jn_moveit_config/config/mycobot_280jn_sim.srdf) | Named targets (`home`, `grasp_approach`, …) |
