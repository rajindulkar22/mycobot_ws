# mycobot_moveit_projects

C++ MoveIt 2 learning nodes for the myCobot 280 JN. These nodes plan and execute arm motions through the same `arm_controller` action server used by the Python sim nodes and RViz.

**Prerequisites:** Gazebo running, then `move_group` (see [workspace README](../README.md#moveit-2)).

**Debugging / history:** [WORKSPACE_DEBUGGING_AND_HISTORY.md](../WORKSPACE_DEBUGGING_AND_HISTORY.md) — sim time, triple `/clock`, FSM vs pick_cube vs cube_approach.

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

MoveIt **vision-guided approach + Cartesian pre-grasp + gripper pick-and-place**. Arm motions use MoveIt; gripper uses `/gripper_action_controller/gripper_cmd`. The launch file auto-starts `color_cube_detector` and `pixel_to_world`.

### Sequence

```
wait for /selected_cube/world_center (10 s timeout)
     → approach (vision XY, grasp_approach orientation)
     → vision hover → vision pre-grasp (Cartesian, cube-relative Z)
     → close gripper → hold 1.5 s → attach pick_cube to gripper_tcp
     → lift vision hover → lift vision carry height
     → plan to place (carry height; MoveIt collision-checks attached cube)
     → place descend → detach cube → update planning scene
     → open gripper → hold 0.5 s
     → retract to carry height → return home
```

`cube_approach` subscribes to **`/selected_cube/world_center`** (from `pixel_to_world`) for pick XY/Z. It adds the table to the MoveIt planning scene at startup, attaches the 25 mm cube after the gripper closes, and detaches it at the release pose before opening.

### Vision pipeline

| Node | Executable | Topics |
|------|------------|--------|
| Detector | `color_cube_detector` | sub: `/overhead_camera/image`; pub: `/selected_cube/pixel_center`, `/selected_cube/annotated_image` |
| Projector | `pixel_to_world` | sub: `/selected_cube/pixel_center`, `/overhead_camera/camera_info`; pub: `/selected_cube/world_center` |
| Pick | `cube_approach` | sub: `/selected_cube/world_center` |

```
/overhead_camera/image  →  color_cube_detector  →  /selected_cube/pixel_center
/overhead_camera/camera_info  +  pixel_center  →  pixel_to_world  →  /selected_cube/world_center
```

Legacy `red_cube_detector` publishes to `/red_cube/pixel_center` — use **`color_cube_detector`** for multi-color picks.

### Run (recommended — launch includes vision)

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash

# Terminal 1 — Gazebo + MoveIt (overhead camera bridged)
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py

# Terminal 2
ros2 run mycobot_sim_projects gripper_commander -- open
ros2 launch mycobot_moveit_projects cube_approach.launch.py target_color:=blue
```

Launch starts vision nodes, waits 2 s, then starts `cube_approach`. **Do not** run a second manual `color_cube_detector` with a different `target_color`.

```bash
ros2 launch mycobot_moveit_projects sort_cubes.launch.py   # red → green → blue sequence
```

### Manual vision (optional)

```bash
ros2 run mycobot_sim_projects color_cube_detector \
  --ros-args -p target_color:=blue -p use_sim_time:=true
ros2 run mycobot_sim_projects pixel_to_world --ros-args -p use_sim_time:=true
ros2 topic echo /selected_cube/world_center --once
ros2 launch mycobot_moveit_projects cube_approach.launch.py
```

### Prerequisites

| Required | Notes |
|----------|-------|
| Gazebo + move_group | Same as other MoveIt C++ nodes |
| Vision | Included in launch, or manual nodes above |
| Gripper open at start | `ros2 run mycobot_sim_projects gripper_commander -- open` if needed |
| Planning scene | `cube_approach` adds `work_table` automatically |
| Matching `target_color` | `red` / `green` / `blue` must match the cube in the scene |

### Launch arguments

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `target_color` | `red` | Cube colour for `color_cube_detector` |
| `place_x` | `0.10` | Place approach TCP X in `world` |
| `place_y` | `0.15` | Place approach TCP Y in `world` |
| `place_z` | `0.140` | Carry height for approach and retract (m) |
| `place_descend_z` | `0.055` | Release height before opening gripper (m) |
| `return_home` | `true` | Plan to SRDF `home` after release |

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py target_color:=green
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  target_color:=blue place_x:=0.10 place_y:=0.15 place_z:=0.140
```

### Expected `world_center` by color

| Color | Expected y (MoveIt world) |
|-------|---------------------------|
| red | ≈ +0.10 |
| green | ≈ 0.00 |
| blue | ≈ -0.10 |

Verify in `cube_approach` log: `Vision cube position: x=..., y=..., z=...`

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
| `/clock` Publisher count **3**, hz ~1500 | Three Gazebo stacks (three `clock_bridge` nodes) | Full cleanup; launch **one** stack only — see below and [WORKSPACE_DEBUGGING_AND_HISTORY.md](../WORKSPACE_DEBUGGING_AND_HISTORY.md) |
| `/clock` Publisher count **1**, hz ~500 | Healthy | Proceed with MoveIt |
| `No cube detection received within 10 seconds` | Vision not running | Launch includes vision; or start `color_cube_detector` + `pixel_to_world`; check `/overhead_camera/image` |
| Robot picks wrong cube | Wrong `target_color` or duplicate detectors | `target_color:=blue` for blue; one detector; check `Vision cube position` log (y ≈ -0.10 for blue) |
| `No executable found` (color_cube_detector) | Stale build | `colcon build --packages-select mycobot_sim_projects`; source install |
| `Detected jump back in time` (TF) | Gazebo restarted while `move_group` still running, or stale `gzserver` | Stop everything, then clean up orphans and restart together |
| MoveIt execute aborted / unknown goal response | Same — sim `/clock` jumped backward, TF buffer cleared | Same |
| Pick fails right after restarting Gazebo only | `move_group` still bound to old sim time | Never leave `move_group` running across a Gazebo restart |
| Gazebo RTF ~4% during grasp only | Mesh finger contact + physics load | Normal under load; not the same as broken `/clock` |

**Verify sim time before every pick:**

```bash
ros2 topic info /clock -v | grep "Publisher count"   # must be 1
ros2 topic hz /clock                                 # ~500 Hz when healthy
```

**Clean shutdown before restart:**

```bash
pkill -9 -f "ros2 launch" || true
pkill -9 -f "ign gazebo" || true
pkill -9 -f "gz sim" || true
pkill -9 -f gzserver || true
pkill -9 -f parameter_bridge || true
pkill -9 -f clock_bridge || true
pkill -f move_group || true

ros2 daemon stop
ros2 daemon start
```

Then launch the combined stack (recommended):

```bash
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py
```

Or restart **both** Gazebo and `move_group` in separate terminals. Sim Workbench **Stop sim stack** runs the same cleanup.

**Before pick:** open gripper → go home → wait ~6 s. Avoid running `gazebo_pose_commander -- home` immediately before `cube_approach` without a pause — both use `arm_controller`.

| Grasp slips / cube drops on lift | Low finger–cube friction, fast descend, or stale Gazebo model | Rebuild `mycobot_280jn_sim`, full `pkill` Gazebo restart, use combined stack. Confirm 25 mm cube (`grep "0.025"` in installed world). Compare with `gazebo_pose_commander -- pick_cube` — if that works but MoveIt does not, restart `move_group` after SRDF changes. |

Full symptom tables: [WORKSPACE_DEBUGGING_AND_HISTORY.md](../WORKSPACE_DEBUGGING_AND_HISTORY.md).

---

## planning_scene_objects

Adds two collision boxes to the MoveIt planning scene in the **world** frame (does not move Gazebo physics objects — only MoveIt's internal collision model):

| ID | Centre (world) | Size (m) |
|----|----------------|----------|
| `work_table` | (0, 0, −0.03) | 0.8 × 0.6 × 0.05 |
| `pick_cube` | (0.15, 0.10, 0.0075) | Red cube default spawn; vision pick uses detected XY |

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
