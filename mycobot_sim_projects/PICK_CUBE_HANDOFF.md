# myCobot Gazebo Pick-Cube — Change Summary (for GPT handoff)

## Context

- **Robot:** myCobot 280 JN with adaptive gripper in Gazebo (ROS 2 Humble, Docker container `mycobot-humble`)
- **Goal:** Reliable **top-down pick** of `pick_cube` on `work_table` via `ros2 run mycobot_sim_projects gazebo_pose_commander -- pick_cube`
- **User constraint:** All grasp/pick logic lives in `gazebo_pose_commander.py`, **not** `manipulation_state_machine.py`

---

## Problems fixed (iterative debugging)

| Issue | Root cause | Fix |
|-------|------------|-----|
| Gripper beside cube (XY offset) | Used side-pick `joint1≈−0.13` with top-down wrist | Restored `joint1≈0.330216` for all top-down poses |
| Cube too big / won't fit gripper | 40 mm cube vs 20–45 mm gripper spec | Shrunk cube to **25 mm** |
| Cube size not updating in sim | `colcon --symlink-install` broke paths between Docker `/root` and host `/home` | Force **real file copy** in CMake install |
| No pickup / gripper too high | `grasp_descend` TCP ~59 mm above cube top | FK retune: descend TCP ~**54 mm** (~34 mm above 25 mm cube top) |
| Table collision on descend | Old `descend_080`/`descend_090` poses too deep | Removed deep descend poses; staged approach → hover → descend |
| Grip works but no lift | Jumped `grasp_descend` → approach in one move (arc hits table) | **Two-stage lift:** `grasp_lift` (hover) → `grasp_retract` (approach) |
| Process hangs after close | `close_gripper()` blocked forever waiting for gripper action `reached_goal` on object contact | **3 s timeout** on close with `proceed_on_timeout=True` |

---

## Files changed

### 1. `gazebo_pose_commander.py` — main file

#### Architecture decision
- **`pick_cube`** is a full scripted sequence: open gripper → arm poses → close → hold → lift
- **`manipulation_state_machine.py`** was reverted to generic demo poses; comment points users to `gazebo_pose_commander.py`

#### Gripper constants
```python
GRIPPER_OPEN = 0.0          # normal open (NOT wide/0.12)
GRIPPER_CLOSED = -0.55      # was -0.65; firm but less hang-prone
GRIPPER_MAX_EFFORT = 8.0    # was 5.0
GRIPPER_HOLD_SEC = 0.8      # dwell after close before lift
GRIPPER_OPEN_TIMEOUT_SEC = 10.0
GRIPPER_CLOSE_TIMEOUT_SEC = 3.0   # don't block forever on stall
```

#### Motion timing
```python
DESCEND_DURATION_SCALE = 0.7
DESCEND_MIN_DURATION_SEC = 3.5    # slow final descend
LIFT_DURATION_SCALE = 0.8
LIFT_MIN_DURATION_SEC = 4.0       # slow lift stages
```

#### Top-down joint poses (radians, order = JOINT_NAMES)
All use **fixed wrist** for vertical pick: `j1≈0.330216, j5≈−0.007679, j6≈−1.240580`; only **j2, j3, j4** change for vertical motion.

| Pose | j2 | j3 | j4 | FK TCP z (approx) | Purpose |
|------|----|----|-----|-------------------|---------|
| `grasp_approach` | −0.212581 | −1.602910 | 0.241554 | ~0.140 m | Safe height above cube |
| `grasp_hover` | −0.407500 | −1.711000 | 0.520000 | ~0.105 m | Mid-descent / 1st lift stage |
| `grasp_descend` | −0.700000 | −1.875000 | 0.938000 | ~0.054 m | Pre-close (~34 mm above cube top) |
| `grasp_lift` | same as hover | | | ~0.105 m | Lift off table (stage 1) |
| `grasp_retract` | same as approach | | | ~0.140 m | Carry height (stage 2) |

Legacy aliases kept:
- `top_pick_approach_calibrated` → `grasp_approach`
- `top_pick_pregrasp_calibrated` → `grasp_descend`

Old side-pick poses (`pick_approach`, `pick_pregrasp`, `pick_test`) retained for reference only.

#### Sequences
```python
GRASP_SEQUENCE = ("grasp_approach", "grasp_hover", "grasp_descend")
LIFT_SEQUENCE  = ("grasp_lift", "grasp_retract")
```

#### Full `pick_cube` flow
```
open → grasp_approach → grasp_hover → grasp_descend → close → hold 0.8s → grasp_lift → grasp_retract
```

#### New / changed methods
- **`execute_pick_cube(duration)`** — runs full sequence above
- **`execute_grasp_approach(duration)`** — arm-only: open → GRASP_SEQUENCE; user closes/lifts manually
- **`_send_gripper_goal(..., timeout_sec, proceed_on_timeout)`** — timed gripper waits with detailed logging
- **`_wait_future(future, timeout_sec)`** — spin_once loop instead of infinite `spin_until_future_complete`
- **`close_gripper()`** uses `proceed_on_timeout=True` so pick continues even if action never reports success on cube contact
- **`_step_duration()`** — longer times for descend, hover, and lift poses

#### Import added
```python
from action_msgs.msg import GoalStatus
```

---

### 2. `mycobot_table.sdf` — world / cube

**`pick_cube` model changes:**
- Size: **40 mm → 25 mm** (`0.025 0.025 0.025`)
- Center Z: **0.4125 m** (table top 0.400 m + half cube 0.0125 m)
- Position: **(0.20, 0.00, 0.4125)** in Gazebo world frame
- Mass: **0.05 kg**
- Friction: **mu = mu2 = 2.5** (increased from 1.0 to reduce slip during lift)
- Contact stiffness: `kp=100000`, `kd=10`

**In robot `joint1` frame** (robot spawned at z=0.405 m):
- Cube center z ≈ **0.0075 m**
- Cube top z ≈ **0.020 m**

---

### 3. `CMakeLists.txt` — install fix

**Problem:** Symlink install pointed to absolute source paths that differ between Docker (`/root/mycobot_ws`) and host (`/home/raj/mycobot_ws`), so Gazebo kept loading old 40 mm cube.

**Fix:**
- Removed `worlds/` from generic `install(DIRECTORY ...)`
- Added explicit `install(FILES worlds/mycobot_table.sdf ...)`
- Added `install(CODE "file(COPY ...")` to force a **real copy** into install tree

**Verify after build:**
```bash
grep "Cube size" /root/mycobot_ws/install/mycobot_280jn_sim/share/mycobot_280jn_sim/worlds/mycobot_table.sdf
# Should show: 0.025 m (25 mm)
```

**Important:** Full Gazebo restart required after world changes.

---

### 4. `manipulation_state_machine.py` — reverted

- Briefly had calibrated top-pick poses wired in; **user asked to revert**
- Now uses generic `home/ready/approach/lift` demo poses only
- Comment directs grasp work to `gazebo_pose_commander.py`

---

### 5. URDF / controllers (context)

`mycobot_280jn_sim.urdf.xacro`:
- Gripper mimic joints with `damping=0.50`, `friction=0.20` on finger joints
- `gripper_tcp` link for FK reference

`controllers.yaml`:
- `gripper_action_controller`: `allow_stalling: true`, `stall_timeout: 1.0`, `goal_tolerance: 0.02`, `max_effort: 5.0`
- Client sends `max_effort=8.0` in goal (can exceed yaml default)

---

## Pick strategy (design rules)

1. **Top-down wrist:** `j5≈0`, `j6≈−71°` — gripper points down
2. **XY alignment:** `joint1≈0.33` centers TCP over cube at (0.20, 0.00)
3. **Vertical motion:** only j2–j4 change between approach/hover/descend/lift
4. **Do not drive TCP into cube volume** — stop above, **close** achieves grasp
5. **Lift mirrors descend:** descend → hover → approach (not descend → approach in one jump)
6. **Open at 0.0 rad**, not `wide` (0.12) — avoids joint limit issues

---

## How to run

```bash
# Terminal 1 — must restart after world changes
source /root/mycobot_ws/install/setup.bash
ros2 launch mycobot_280jn_sim gazebo_sim.launch.py

# Terminal 2
source /root/mycobot_ws/install/setup.bash
ros2 run mycobot_sim_projects gazebo_pose_commander -- pick_cube
```

**Rebuild:**
```bash
colcon build --packages-select mycobot_280jn_sim mycobot_sim_projects
```

**Manual debug steps** (partial sequence):
```bash
ros2 run mycobot_sim_projects gazebo_pose_commander -- grasp_approach
ros2 run mycobot_sim_projects gazebo_pose_commander -- grasp_hover
ros2 run mycobot_sim_projects gazebo_pose_commander -- grasp_descend
ros2 run mycobot_sim_projects gripper_commander -- close
ros2 run mycobot_sim_projects gazebo_pose_commander -- grasp_lift
ros2 run mycobot_sim_projects gazebo_pose_commander -- grasp_retract
```

---

## Expected log output (successful run)

```
Running top-down pick: grasp_approach → grasp_hover → grasp_descend → close → grasp_lift → grasp_retract
... Pose 'grasp_approach' completed ...
... Pose 'grasp_hover' completed ...
... Pose 'grasp_descend' completed ...
Gripper 'close': position=-0.55 rad ...
Gripper 'close' accepted; waiting up to 3.0 s ...
(Holding grasp for 0.8 s ...)          # or timeout warning then continue
... Pose 'grasp_lift' completed ...
... Pose 'grasp_retract' completed ...
pick_cube completed (approach → descend → close → lift → retract).
```

---

## Known remaining tuning knobs

If pick still fails:
- **Descend too high:** lower j2/j3/j4 slightly (watch table collision)
- **Descend collision:** raise `grasp_descend` ~2 mm
- **Slip on lift:** increase `GRIPPER_CLOSED` toward −0.65 or cube friction
- **Close timeout every time:** normal with object contact; lift should still proceed
- **Cube knocked over:** reset sim or reposition `pick_cube`

---

## Copy-paste prompt for GPT

> We tuned a ROS 2 Humble Gazebo simulation for myCobot 280 JN adaptive gripper top-down pick of a 25 mm red cube at world pose (0.20, 0.00, 0.4125). All pick logic is in `mycobot_sim_projects/gazebo_pose_commander.py` via `pick_cube` command. Sequence: open(0.0) → grasp_approach → grasp_hover → grasp_descend (TCP z≈0.054m) → close(−0.55 rad, 8N, 3s timeout proceed_on_timeout) → hold 0.8s → grasp_lift (=hover) → grasp_retract (=approach). joint1=0.330216 for XY; j5/j6 fixed for top-down. Cube shrunk to 25mm in `mycobot_table.sdf` with mu=2.5; CMakeLists forces real copy of world file (no broken symlinks). manipulation_state_machine.py is NOT used for pick. Help me [YOUR NEXT TASK].
