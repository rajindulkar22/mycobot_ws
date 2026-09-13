# mycobot_sim_projects

Python ROS 2 nodes for myCobot 280 JN simulation: pose control, gripper, state machines, **computer vision**, and **YOLO11n** cube detection.

**Workspace hub:** [../README.md](../README.md)  
**Vision theory (steps 14–15):** [THEORY.md](THEORY.md)  
**YOLO / ML pipeline (step 17):** [YOLO_VISION_GUIDED_PICK_AND_PLACE.md](YOLO_VISION_GUIDED_PICK_AND_PLACE.md)

---

## Vision and AI nodes

| Node | Type | Purpose |
|------|------|---------|
| `color_cube_detector` | Classical CV (HSV) | Segment red/green/blue cubes; publish `/selected_cube/pixel_center` |
| `yolo_cube_detector` | **YOLO11n (Ultralytics)** | Trained multi-class detector; same topics as HSV node |
| `generate_yolo_dataset` | Dataset generator | Randomize Gazebo cube poses; auto-label for YOLO training |
| `pixel_to_world` | Geometry | Pinhole projection → `/selected_cube/world_center` |

Trained weights and dataset: [`mycobot_yolo_assets`](../mycobot_yolo_assets/) package (committed under `src/`).

### Run YOLO detector (Gazebo required)

```bash
ros2 run mycobot_sim_projects yolo_cube_detector \
  --ros-args -p use_sim_time:=true -p target_color:=blue
ros2 topic echo /selected_cube/pixel_center --once
```

### Generate training data

```bash
ros2 run mycobot_sim_projects generate_yolo_dataset \
  --ros-args -p use_sim_time:=true -p samples:=10
```

Demo video: [../mycobot_yolo_assets/media/yolo_dataset_generation.webm](../mycobot_yolo_assets/media/yolo_dataset_generation.webm)

---

## Pick and manipulation nodes

| Node | Purpose |
|------|---------|
| `gazebo_pose_commander` | Calibrated fixed-pose pick (`pick_cube`) |
| `gripper_commander` | Open/close gripper |
| `manipulation_state_machine` / `_ui` | Generic FSM demo (not for cube pick) |
| `sim_workbench_ui` | GUI for Gazebo + MoveIt + `cube_approach` |

Vision-guided MoveIt pick: use [`cube_approach.launch.py`](../mycobot_moveit_projects/launch/cube_approach.launch.py) (YOLO default).

---

## Build

```bash
colcon build --symlink-install --packages-select \
  mycobot_yolo_assets mycobot_sim_projects
source install/setup.bash
```

YOLO inference requires `/root/yolo_env` (PyTorch + Ultralytics) — see [YOLO handoff](YOLO_VISION_GUIDED_PICK_AND_PLACE.md#install-yolo).
