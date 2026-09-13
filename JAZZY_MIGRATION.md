# ROS 2 Jazzy migration guide

You are on the **`jazzy`** branch. Stable Humble development stays on **`main`** with Docker container `mycobot-humble`.

| | Humble (`main`) | Jazzy (`jazzy`) |
|---|---|---|
| ROS distro | Humble | Jazzy |
| Gazebo | Fortress (`ign` CLI) | Harmonic (`gz` CLI) |
| Docker image | `osrf/ros:humble-desktop-full-jammy` | `osrf/ros:jazzy-desktop-full-noble` |
| Container | `mycobot-humble` | `mycobot-jazzy` |
| Source line | `source /opt/ros/humble/setup.bash` | `source /opt/ros/jazzy/setup.bash` |
| Python (YOLO venv) | 3.10 | 3.12 |

Both containers bind-mount the same workspace: `~/mycobot_ws` ↔ `/root/mycobot_ws`.

**Never source Humble and Jazzy in the same shell.** When switching git branches, remove mixed build artifacts:

```bash
rm -rf ~/mycobot_ws/build ~/mycobot_ws/install ~/mycobot_ws/log
```

---

## 1. Create the Jazzy container (once)

On the host:

```bash
xhost +local:docker
bash ~/mycobot_ws/src/scripts/create-mycobot-jazzy-container.sh
```

Or manually:

```bash
docker run -it \
  --name mycobot-jazzy \
  --network host --ipc host --device=/dev/dri \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/mycobot_ws:/root/mycobot_ws \
  osrf/ros:jazzy-desktop-full-noble
```

Start an existing container:

```bash
docker start -ai mycobot-jazzy
```

---

## 2. Install dependencies (inside container)

```bash
source /opt/ros/jazzy/setup.bash
bash /root/mycobot_ws/src/scripts/jazzy-docker-deps.sh
```

This installs `ros-jazzy-ros-gz`, MoveIt, ros2_control, colcon, and related packages. **Run this before the first build** — skipping it causes errors like missing `control_msgs` or `moveit_ros_planning_interface`.

If `git checkout jazzy` fails with *dubious ownership* inside the container:

```bash
git config --global --add safe.directory /root/mycobot_ws/src
```

### mycobot_description

Official [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) has no Jazzy branch. This workspace vendors `mycobot_ros2` (Humble branch) under `src/mycobot_ros2/`. If `rosdep` or build fails, install missing deps manually or pin a known-good commit.

---

## 3. YOLO virtual environment (Python 3.12)

Inside `mycobot-jazzy`:

```bash
bash /root/mycobot_ws/src/scripts/yolo-env-jazzy.sh
source /root/yolo_env/bin/activate
```

`yolo_cube_detector` auto-reexecs into `/root/yolo_env` when Ultralytics is not on the system Python.

---

## 4. Build

```bash
source /opt/ros/jazzy/setup.bash
cd /root/mycobot_ws

# Upstream description packages (Humble branch) build on Jazzy:
colcon build --symlink-install --packages-select \
  mycobot_description mycobot_interfaces mycobot_280jn
source install/setup.bash

colcon build --symlink-install --packages-select \
  mycobot_yolo_assets mycobot_280jn_sim mycobot_sim_projects \
  mycobot_280jn_moveit_config mycobot_moveit_projects
source install/setup.bash
```

**Build only inside the container** (or only on the host) for `build_jazzy` / `install_jazzy`. Mixing host and container paths breaks CMake caches. Before the first container build:

```bash
rm -rf /root/mycobot_ws/build_jazzy /root/mycobot_ws/install_jazzy /root/mycobot_ws/log_jazzy
export COLCON_LOG_PATH=/root/mycobot_ws/log_jazzy
colcon build --symlink-install \
  --build-base build_jazzy --install-base install_jazzy \
  --packages-select mycobot_description mycobot_280jn ...
```

If MoveIt planning fails at runtime, regenerate SRDF/OMPL with MoveIt Setup Assistant inside this container and commit config changes on the `jazzy` branch only.

---

## 5. Jazzy-specific code changes (this branch)

| Area | Change |
|---|---|
| `mycobot_280jn_sim/worlds/mycobot_table.sdf` | `gz-physics-dartsim-plugin` (Harmonic) |
| `mycobot_280jn_sim/launch/gazebo_sim.launch.py` | `GZ_SIM_RESOURCE_PATH` preferred for meshes |
| `generate_yolo_dataset.py` | `gz service` + `gz.msgs.*` |
| Dataset / debug docs | `gz topic`, `gz service` |

---

## 6. Smoke tests

**Terminal 1 — stack**

```bash
source /opt/ros/jazzy/setup.bash
source /root/mycobot_ws/install_jazzy/setup.bash
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_stack.launch.py
```

Pass: `/clock` publishing, `/overhead_camera/image` visible.

**Terminal 2 — YOLO detector**

```bash
ros2 run mycobot_sim_projects yolo_cube_detector
```

Pass: `/selected_cube/pixel_center`, annotated image topic.

**Terminal 3 — single pick**

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py target_color:=blue
```

Pass: log line `Pick-and-place complete`.

**Terminal 4 — sort all colours**

```bash
ros2 launch mycobot_moveit_projects sort_cubes.launch.py
```

Pass: logs in `/tmp/sort_red_pick.log`, `/tmp/sort_green_pick.log`, `/tmp/sort_blue_pick.log` each contain `Pick-and-place complete`.

**Dataset generation (10 samples)**

```bash
ros2 run mycobot_sim_projects generate_yolo_dataset --ros-args -p samples:=10
```

Pass: `gz service` set_pose succeeds; images and labels under `mycobot_yolo_assets/assets/dataset/`.

### Gazebo CLI sanity check

```bash
gz service -s /world/mycobot_world/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req '
name: "pick_cube"
position { x: 0.10 y: 0.10 z: 0.4125 }
orientation { w: 1.0 }'
```

Expected: `data: true`.

---

## 7. Troubleshooting launch failures

| Symptom | Fix |
|---|---|
| `package 'controller_manager' not found` | Run `bash /root/mycobot_ws/src/scripts/jazzy-docker-deps.sh` or `apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control` |
| `Failed to load system plugin [gz_ros2_control-system]` | Same — `ros-jazzy-gz-ros2-control` not installed |
| Arm collapsed / no joint motion | Controllers never started (see above); relaunch after installing deps |
| `failed to load driver: nvidia-drm` in Docker | Usually harmless if the Gazebo window still opens; sim physics runs |

After installing missing apt packages, **restart the launch** (Ctrl+C, then run again).

---

## 8. Status checklist

| Component | Status |
|---|---|
| Gazebo Harmonic world load | Passed (headless `gz sim`, camera topics) |
| `colcon build` (all packages) | Passed on host Jazzy (MoveIt API fixes applied) |
| ros_gz bridge + MoveIt stack | Verify full stack in `mycobot-jazzy` container |
| YOLO detect + pick (`cube_approach`) | Verify in container with `/root/yolo_env` |
| Sort sequence (`sort_cubes`) | Verify in container |
| Dataset generator (`gz service`) | Code ported; verify with Gazebo running |

---

## 9. Branch workflow

```bash
cd ~/mycobot_ws/src
git checkout main    # use mycobot-humble, rebuild for Humble
git checkout jazzy   # use mycobot-jazzy, rebuild for Jazzy
```

Optional draft PR: `jazzy` → `main` (do not merge until full regression passes).
