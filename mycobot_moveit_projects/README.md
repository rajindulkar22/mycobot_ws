# mycobot_moveit_projects

C++ MoveIt 2 learning nodes for the myCobot 280 JN. These nodes plan and execute arm motions through the same `arm_controller` action server used by the Python sim nodes and RViz.

**Prerequisites:** Gazebo running, then `move_group` (see [workspace README](../README.md#moveit-2)).

---

## Nodes

| Executable | Launch file | Purpose |
|------------|-------------|---------|
| `named_targets` | `named_targets.launch.py` | Plan/execute SRDF named states (`home` → `grasp_approach` → `home`) |
| `pose_target` | `pose_target.launch.py` | FK read TCP → Z−3 cm → IK → OMPL → execute |
| `planning_scene_objects` | *(run directly)* | Add table + 25 mm cube boxes to MoveIt planning scene |
| `test_obstacle` | *(run directly)* | Add/remove a 10 cm box at the grasp_approach TCP pose |

---

## How to run

Inside Docker (after sourcing ROS and the workspace):

```bash
# Terminal 1 — Gazebo + controllers
ros2 launch mycobot_280jn_sim gazebo_sim.launch.py

# Terminal 2 — move_group
ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py

# Terminal 3 — pick a demo
ros2 launch mycobot_moveit_projects named_targets.launch.py
# or
ros2 launch mycobot_moveit_projects pose_target.launch.py
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

## planning_scene_objects

Adds two collision boxes to the MoveIt planning scene in the **world** frame (does not move Gazebo physics objects — only MoveIt's internal collision model):

| ID | Centre (world) | Size (m) |
|----|----------------|----------|
| `work_table` | (0, 0, −0.03) | 0.8 × 0.6 × 0.05 |
| `pick_cube` | (0.20, 0, 0.0075) | 0.025 cube |

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
