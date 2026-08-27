# myCobot 280 Jetson Nano — ROS 2 Humble Docker Simulation Guide

For workspace packages, commands, pick-and-place workflow, and recent changes, see [README.md](README.md).

This guide records the complete environment created for running the Elephant Robotics `mycobot_ros2` repository on an Ubuntu 24.04 host while keeping ROS 2 Jazzy installed locally. ROS 2 Humble runs inside an Ubuntu 22.04 Docker container, while Cursor edits the workspace directly from the host.

## 1. Environment architecture

| Component | Configuration |
|---|---|
| Host operating system | Ubuntu 24.04 |
| Host ROS version | ROS 2 Jazzy |
| Container operating system | Ubuntu 22.04 Jammy |
| Container ROS version | ROS 2 Humble |
| Docker image | `osrf/ros:humble-desktop-full-jammy` |
| Docker container | `mycobot-humble` |
| Host workspace | `/home/raj/mycobot_ws` |
| Container workspace | `/root/mycobot_ws` |
| Repository branch | `humble` |
| Robot package | `mycobot_280jn` |

The host workspace is bind-mounted into the container:

```text
/home/raj/mycobot_ws  <-->  /root/mycobot_ws
```

Consequently, files saved in Cursor on Ubuntu are immediately visible inside Docker.

## 2. Commands that run on the host

Host commands use a prompt similar to:

```text
raj@RAJ:~$
```

Do not run Humble package commands at this prompt. The host only knows the locally installed ROS 2 Jazzy packages.

### Allow Docker GUI windows

RViz and other graphical programs inside Docker need access to the host display:

```bash
xhost +local:docker
```

The expected response is:

```text
non-network local connections being added to access control list
```

### Create the persistent workspace

```bash
mkdir -p ~/mycobot_ws/src
```

### Create and start the container for the first time

Run this only once, because it creates a container named `mycobot-humble`:

```bash
docker run -it \
  --name mycobot-humble \
  --network host \
  --ipc host \
  --device=/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/mycobot_ws:/root/mycobot_ws \
  osrf/ros:humble-desktop-full-jammy
```

Important options:

| Option | Purpose |
|---|---|
| `--name mycobot-humble` | Assigns a reusable container name |
| `--network host` | Simplifies ROS 2 discovery and networking |
| `--ipc host` | Helps GUI/shared-memory applications |
| `--device=/dev/dri` | Gives the container access to graphics rendering |
| `DISPLAY` and `/tmp/.X11-unix` | Display RViz on the host desktop |
| Workspace volume | Preserves source code outside the container |

### Start the existing container later

Do not repeat `docker run`. Restart the existing container with:

```bash
xhost +local:docker
docker start -ai mycobot-humble
```

### Open another terminal in the running container

```bash
docker exec -it mycobot-humble bash
```

### Stop the container

From another host terminal:

```bash
docker stop mycobot-humble
```

Alternatively, type `exit` at the main interactive container prompt.

### Revoke GUI permission when finished

```bash
xhost -local:docker
```

## 3. Commands that run inside Docker

Container commands use a prompt similar to:

```text
root@RAJ:/#
```

### Test GUI forwarding

```bash
rviz2
```

If RViz opens, GUI forwarding works. Close it using `Ctrl+C` in the terminal.

If rendering fails, try:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

### Install development dependencies

```bash
apt update

apt install -y \
  git \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher
```

Initialize rosdep:

```bash
rosdep init
rosdep update
```

If `rosdep init` reports that it is already initialized, run only `rosdep update`.

### Clone the repository

```bash
cd /root/mycobot_ws/src

git clone -b humble --depth 1 \
  https://github.com/elephantrobotics/mycobot_ros2.git
```

Repository: <https://github.com/elephantrobotics/mycobot_ros2>

### Install package dependencies

```bash
cd /root/mycobot_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y
```

### Build the workspace

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify the packages:

```bash
ros2 pkg list | grep mycobot
```

## 4. Important: source ROS in every new container terminal

Every new `docker exec` terminal must load both environments:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
```

To make this automatic inside the container:

```bash
echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
echo 'source /root/mycobot_ws/install/setup.bash' >> /root/.bashrc
```

These commands should be added only once.

## 5. First simulation: RViz joint sliders

### Basic robot

Inside Docker:

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mycobot_280jn slider_control.launch.py
```

This launches the myCobot 280 Jetson Nano model and the joint slider interface. It is a kinematic visualization, not a physics simulation.

### Robot with adaptive gripper

```bash
ros2 launch mycobot_280jn \
  slider_control_adaptive_gripper.launch.py
```

Move the sliders slowly and check that each corresponding joint moves in RViz.

### Inspect joint data

Keep the simulation running. From a second host terminal:

```bash
docker exec -it mycobot-humble bash
```

Then, inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 topic list
ros2 topic echo /joint_states
```

Moving a slider should change the positions printed on `/joint_states`.

## 6. Adaptive-gripper URDF correction

The Humble branch contained malformed XML in:

```text
mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_adaptive_gripper.urdf
```

The invalid line was:

```xml
<limit effort = "1000.0" lower = "-2.932"1 upper = "2.9321" velocity = "0"/>
```

The corrected line is:

```xml
<limit effort = "1000.0" lower = "-2.9321" upper = "2.9321" velocity = "0"/>
```

The correction command, run inside Docker, was:

```bash
sed -i 's/lower = "-2.932"1 upper/lower = "-2.9321" upper/' \
  /root/mycobot_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_adaptive_gripper.urdf
```

Verify it:

```bash
grep -n '2.932' \
  /root/mycobot_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn_adaptive_gripper.urdf
```

Rebuild the affected packages:

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select mycobot_description mycobot_280jn

source install/setup.bash
```

Then launch the adaptive-gripper model again.

## 7. Edit the workspace using local Cursor

Because the workspace is bind-mounted, open the host directory in Cursor:

```bash
cursor ~/mycobot_ws
```

If the terminal command is unavailable, open Cursor and select **File → Open Folder**, then choose:

```text
/home/raj/mycobot_ws
```

If files are owned by root because they were created inside Docker, run this on the host:

```bash
sudo chown -R "$USER":"$USER" ~/mycobot_ws
```

Always edit source files under:

```text
~/mycobot_ws/src/mycobot_ros2
```

Do not manually edit generated files under:

```text
~/mycobot_ws/build
~/mycobot_ws/install
~/mycobot_ws/log
```

### Cursor-to-Docker development cycle

1. Edit and save a source file in Cursor.
2. Enter the running container using `docker exec`.
3. Build the affected ROS package.
4. Source `install/setup.bash`.
5. Launch and test.

Example rebuild:

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select mycobot_description mycobot_280jn

source install/setup.bash
```

## 8. Avoid mixing the host and container ROS installations

The following host-side error is expected if a Humble package is launched outside Docker:

```text
Package 'mycobot_280jn' not found, searching: ['/opt/ros/jazzy']
```

It means the command was run on the Ubuntu 24.04/Jazzy host. Enter Docker and rerun it:

```bash
docker exec -it mycobot-humble bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 launch mycobot_280jn slider_control.launch.py
```

## 9. Recovering when a `docker` command was run inside the container

`docker` itself is a host-side tool. If a `docker ...` command (for example `docker exec`) is typed at the container prompt:

```text
root@RAJ:/#
```

it fails, because the container has no access to the host's Docker daemon. Docker commands must always be issued from the Ubuntu host prompt:

```text
raj@RAJ:~$
```

Recover with this exact sequence.

### 1. Exit the container

```bash
exit
```

The prompt should return to:

```text
raj@RAJ:~$
```

### 2. Authorize GUI access on the host

```bash
xhost +SI:localuser:root
```

This is equivalent to the `xhost +local:docker` command in [Section 2](#2-commands-that-run-on-the-host), scoped to the container's root user instead of all local Docker clients.

### 3. Start the container without attaching

```bash
docker start mycobot-humble
```

Expected output:

```text
mycobot-humble
```

Unlike `docker start -ai mycobot-humble` (used in the daily quick-start checklist), this starts the container in the background without attaching a terminal.

### 4. Enter it with the current display

```bash
docker exec -it \
  -e DISPLAY="$DISPLAY" \
  mycobot-humble bash
```

The prompt becomes:

```text
root@RAJ:/#
```

### 5. Source and launch inside Docker

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash

ros2 launch mycobot_sim_projects \
  display_rviz.launch.py
```

## 10. Useful diagnostic commands

Run inside Docker unless stated otherwise.

```bash
# Check the ROS distribution
echo $ROS_DISTRO

# Find myCobot packages
ros2 pkg list | grep mycobot

# List nodes and topics
ros2 node list
ros2 topic list

# Observe joint states
ros2 topic echo /joint_states

# Find launch files
find /root/mycobot_ws/src/mycobot_ros2 \
  -type f -path '*/launch/*' | sort

# Find Jetson Nano launch files
find /root/mycobot_ws/src/mycobot_ros2 \
  -type f -path '*/launch/*' | \
  grep -Ei '280.*jn|jn.*280|jetson|slider|rviz'

# Show container status (run on host)
docker ps -a --filter name=mycobot-humble
```

## 11. Daily quick-start checklist

On the host:

```bash
xhost +local:docker
docker start mycobot-humble
docker exec -it mycobot-humble bash
```

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 launch mycobot_280jn slider_control.launch.py
```

For the adaptive gripper:

```bash
ros2 launch mycobot_280jn \
  slider_control_adaptive_gripper.launch.py
```

## 12. Recommended simulation learning order

1. Visualize the URDF in RViz. — **done** (`display_rviz.launch.py`)
2. Move every joint using the slider GUI. — **done**
3. Observe `/joint_states` and TF frames. — **done** (`joint_monitor`, `tf_explorer`)
4. Write a Python ROS 2 node for predefined joint poses. — **done** (`pose_sequence`, `gazebo_pose_commander`)
5. Add Gazebo physics and `ros2_control`. — **done** (`mycobot_280jn_sim`, `gazebo_sim.launch.py`)
6. Configure MoveIt 2 motion planning. — **in progress** (`mycobot_280jn_moveit_config`; SRDF collision tuning ongoing)
7. Implement fixed-position pick-and-place. — **done** (`gazebo_pose_commander -- pick_cube`)
8. Add collision objects and obstacle avoidance.
9. Add a simulated RGB or depth camera.
10. Integrate OpenCV and YOLO.
11. Explore imitation learning and reinforcement learning.

The slider and RViz projects are the correct early exercises. Gazebo, hand-tuned pick, and MoveIt build on that foundation — see [README.md](README.md) for current run commands.

## 13. Current state summary

- Docker GUI forwarding works; RViz and Tkinter UIs open from the container when `DISPLAY` is set.
- The persistent `mycobot_ws` workspace is bind-mounted from the host.
- The `humble` branch of `elephantrobotics/mycobot_ros2` is installed.
- **Gazebo simulation** runs via `mycobot_280jn_sim` (table, 25 mm `pick_cube`, adaptive gripper, `ros2_control`).
- **Calibrated pick-and-lift** runs via `ros2 run mycobot_sim_projects gazebo_pose_commander -- pick_cube`.
- **MoveIt 2** config exists in `mycobot_280jn_moveit_config` (plan in RViz, execute on the same Gazebo `arm_controller`).
- **Demo state machine** available as CLI (`manipulation_state_machine`) and Tkinter UI (`manipulation_state_machine_ui`).
- `mycobot_sim_projects` also provides RViz demos, keyboard control, and gripper commanders.
- Builds and ROS 2 Humble launch commands must run inside Docker.
- `docker` commands (start/exec/stop) must run on the host prompt, never inside the container; see [Section 9](#9-recovering-when-a-docker-command-was-run-inside-the-container).

Full command reference and changelog: [README.md](README.md).

## 14. Gazebo simulation workflow

Run inside Docker after sourcing both setup files:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
```

### Build (after editing sim or Python packages)

```bash
cd /root/mycobot_ws
colcon build --packages-select mycobot_280jn_sim mycobot_sim_projects mycobot_280jn_moveit_config
```

### Launch Gazebo + controllers

```bash
ros2 launch mycobot_280jn_sim gazebo_sim.launch.py
```

### Pick the cube (second terminal)

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 run mycobot_sim_projects gazebo_pose_commander -- pick_cube
```

### MoveIt (optional — Gazebo must already be running)

Terminal 2:

```bash
ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py
```

Terminal 3:

```bash
ros2 launch mycobot_280jn_moveit_config gazebo_moveit_rviz.launch.py
```

### Verify world file installed correctly

```bash
grep "Cube size" /root/mycobot_ws/install/mycobot_280jn_sim/share/mycobot_280jn_sim/worlds/mycobot_table.sdf
```

Expected: `0.025 m (25 mm ...)`. If the value is wrong or Gazebo still shows a large cube, rebuild `mycobot_280jn_sim` and **fully restart** Gazebo.

### File permissions (host editing)

If MoveIt config files were generated inside Docker, they may be owned by `nobody` on the host and cannot be saved from Cursor. Fix on the host:

```bash
sudo chown -R $USER:$USER ~/mycobot_ws/src/mycobot_280jn_moveit_config
```
