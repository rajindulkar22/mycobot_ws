# Project 17 — YOLO Vision-Guided Pick-and-Place

> **Workspace hub:** [README.md](../README.md#yolo-vision-guided-pick-and-place-step-17) — quick commands and metrics summary.  
> This document is the full validated handoff for install, dataset generation, training, validation, live detection, and runtime pick-and-place.

## Overview

This project replaces HSV-only cube detection with a trained YOLO11n detector. An overhead Gazebo camera detects red, green, and blue cubes; ROS 2 converts the selected detection from pixels to the MoveIt `world` frame; and the existing MoveIt node approaches, grasps, lifts, places, and releases the requested cube.

### Validated stack

- ROS 2 Humble on Ubuntu 22.04 in Docker
- Gazebo Fortress / Ignition Gazebo
- MoveIt 2 and `ros2_control`
- OpenCV, `cv_bridge`, PyTorch CPU, Ultralytics YOLO11n
- myCobot 280 JN with adaptive gripper

### Pipeline

```text
/overhead_camera/image
        ↓
yolo_cube_detector
        ↓ /selected_cube/pixel_center
pixel_to_world
        ↓ /selected_cube/world_center
cube_approach (MoveIt)
        ↓
approach → descend → close → lift → place → open → home
```

YOLO classes:

| ID | Class |
| ---: | --- |
| 0 | `red_cube` |
| 1 | `green_cube` |
| 2 | `blue_cube` |

## Important paths

All YOLO artifacts are in the **`mycobot_yolo_assets`** ROS package (version-controlled under `src/`):

```text
/root/mycobot_ws/src/mycobot_yolo_assets/
├── assets/
│   ├── yolo11n.pt
│   ├── dataset/                    # images, labels, data.yaml
│   └── runs/cube_detector/         # best.pt, last.pt, metrics, plots
├── mycobot_yolo_assets/paths.py    # default_model_path(), default_dataset_dir(), ...
└── setup.py                        # installs assets/ to share/

/root/yolo_env                       # Python venv (not in git)
```

Runtime nodes resolve paths via `mycobot_yolo_assets.paths` (installed share directory). After `colcon build --symlink-install`, writes go back to `src/mycobot_yolo_assets/assets/` for committing.

Related code:

```text
/root/mycobot_ws/src/mycobot_280jn_sim/worlds/mycobot_table.sdf
/root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/generate_yolo_dataset.py
/root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/yolo_cube_detector.py
/root/mycobot_ws/src/mycobot_moveit_projects/launch/cube_approach.launch.py
```

## Camera and scene configuration

The camera directly above the robot was occluded during grasping. The validated shifted camera pose is:

```xml
<pose>0.30 -0.25 1.30 0 1.5708 0</pose>
```

The world needs the sensor system:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Camera sensor:

```xml
<model name="overhead_camera">
  <static>true</static>
  <pose>0.30 -0.25 1.30 0 1.5708 0</pose>
  <link name="camera_link">
    <sensor name="rgb_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <topic>/overhead_camera/image</topic>
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip><near>0.05</near><far>5.0</far></clip>
      </camera>
    </sensor>
  </link>
</model>
```

Camera bridge in `gazebo_sim.launch.py`:

```python
camera_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="overhead_camera_bridge",
    output="screen",
    arguments=[
        "/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    ],
)
```

Verified intrinsics: `640×480`, `fx=554.2546463`, `fy=554.2547035`, `cx=320`, `cy=240`, zero distortion.

The robot is spawned at Gazebo `z=0.405 m`; therefore MoveIt uses `z_moveit = z_gazebo - 0.405`. Camera and cube heights in MoveIt are `0.895 m` and `0.0075 m` respectively. The existing calibrated `pixel_to_world.py` is retained unchanged.

## Install YOLO

```bash
apt update
apt install -y python3-pip python3-venv
python3 -m venv --system-site-packages /root/yolo_env
source /root/yolo_env/bin/activate
python3 -m pip install "numpy<2"
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install ultralytics
```

If a download reports a hash mismatch, never disable verification. Retry safely:

```bash
python3 -m pip cache purge
python3 -m pip install --no-cache-dir --retries 10 --timeout 120 \
  "numpy>=1.23.5,<2" ultralytics
```

Verify:

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
source /root/yolo_env/bin/activate
python3 - <<'PY'
import torch, ultralytics, cv2
from cv_bridge import CvBridge
print("PyTorch:", torch.__version__)
print("CUDA:", torch.cuda.is_available())
print("Ultralytics:", ultralytics.__version__)
print("OpenCV:", cv2.__version__)
print("cv_bridge: OK")
PY
```

Validated: PyTorch `2.14.0+cpu`, CUDA `False`, Ultralytics `8.4.143`, OpenCV `4.11.0`.

## Gazebo pose-control test

With Gazebo running:

```bash
ign service -i -s /world/mycobot_world/set_pose
ign service -s /world/mycobot_world/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 3000 --req '
name: "pick_cube"
position { x: 0.10 y: 0.10 z: 0.4125 }
orientation { w: 1.0 }'
```

Expected response: `data: true`.

## Dataset generator

The generator randomizes the three Gazebo cube poses, obtains a fresh camera image, uses HSV only for automatic annotation, and writes normalized YOLO labels. Runtime detection uses YOLO, not HSV.

Save the following as `generate_yolo_dataset.py`:

```python
#!/usr/bin/env python3
import math
import random
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDatasetGenerator(Node):
    CLASS_IDS = {"red": 0, "green": 1, "blue": 2}
    MODEL_NAMES = {
        "red": "pick_cube",
        "green": "green_cube",
        "blue": "blue_cube",
    }
    HSV_RANGES = {
        "red": [
            (np.array([0, 90, 50]), np.array([12, 255, 255])),
            (np.array([165, 90, 50]), np.array([179, 255, 255])),
        ],
        "green": [(np.array([35, 70, 40]), np.array([90, 255, 255]))],
        "blue": [(np.array([90, 70, 40]), np.array([140, 255, 255]))],
    }

    def __init__(self):
        super().__init__("generate_yolo_dataset")
        self.declare_parameter("samples", 10)
        self.declare_parameter("output_directory", "/root/yolo_cube_dataset")
        self.sample_count = int(self.get_parameter("samples").value)
        self.output_directory = Path(
            self.get_parameter("output_directory").value
        )
        self.bridge = CvBridge()
        self.latest_image = None
        self.frame_number = 0
        self.create_subscription(
            Image,
            "/overhead_camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_x, self.camera_y = 0.30, -0.25
        self.camera_z, self.object_z = 1.30, 0.4125
        self.fx, self.fy = 554.2546463012695, 554.2547035217285
        self.cx, self.cy = 320.0, 240.0
        self.kernel = np.ones((3, 3), np.uint8)
        self.prepare_directories()

    def prepare_directories(self):
        for directory in (
            "images/train", "images/val", "labels/train",
            "labels/val", "previews",
        ):
            (self.output_directory / directory).mkdir(parents=True, exist_ok=True)
        (self.output_directory / "data.yaml").write_text(
            f"path: {self.output_directory}\n"
            "train: images/train\nval: images/val\n\n"
            "names:\n  0: red_cube\n  1: green_cube\n  2: blue_cube\n"
        )

    def image_callback(self, message):
        self.latest_image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        self.frame_number += 1

    def set_cube_pose(self, model_name, x, y, yaw):
        request = (
            f'name: "{model_name}" position {{ x: {x:.6f} y: {y:.6f} '
            f'z: {self.object_z:.6f} }} orientation {{ '
            f'z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f} }}'
        )
        result = subprocess.run(
            [
                "ign", "service", "-s", "/world/mycobot_world/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "3000", "--req", request,
            ],
            capture_output=True, text=True, timeout=5.0, check=False,
        )
        return result.returncode == 0 and "data: true" in result.stdout

    def generate_positions(self):
        positions = {}
        for color in self.CLASS_IDS:
            for _ in range(100):
                candidate = (random.uniform(0.08, 0.28),
                             random.uniform(-0.18, 0.18))
                if all(math.hypot(candidate[0] - x, candidate[1] - y) >= 0.065
                       for x, y in positions.values()):
                    positions[color] = candidate
                    break
            else:
                raise RuntimeError("Could not generate separated positions")
        return positions

    def projected_pixel(self, world_x, world_y):
        depth = self.camera_z - self.object_z
        u = self.cx - (world_y - self.camera_y) * self.fx / depth
        v = self.cy - (world_x - self.camera_x) * self.fy / depth
        return u, v

    def find_cube_box(self, image, color, expected_x, expected_y):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lower, upper in self.HSV_RANGES[color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        expected_u, expected_v = self.projected_pixel(expected_x, expected_y)
        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) < 30:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            u = moments["m10"] / moments["m00"]
            v = moments["m01"] / moments["m00"]
            candidates.append((math.hypot(u - expected_u, v - expected_v), contour))
        if not candidates:
            return None
        distance, contour = min(candidates, key=lambda item: item[0])
        if distance > 50:
            return None
        x, y, width, height = cv2.boundingRect(contour)
        image_height, image_width = image.shape[:2]
        return (
            max(0, x - 2), max(0, y - 2),
            min(image_width - 1, x + width + 2),
            min(image_height - 1, y + height + 2),
        )

    @staticmethod
    def yolo_label(class_id, box, image_width, image_height):
        x1, y1, x2, y2 = box
        return (
            f"{class_id} {((x1+x2)/2)/image_width:.6f} "
            f"{((y1+y2)/2)/image_height:.6f} "
            f"{(x2-x1)/image_width:.6f} {(y2-y1)/image_height:.6f}"
        )

    def wait_for_camera_frames(self, number_of_frames=1):
        target = self.frame_number + number_of_frames
        deadline = time.monotonic() + 120.0
        while rclpy.ok() and self.frame_number < target:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for fresh camera frames")

    def collect(self):
        self.get_logger().info("Waiting for the overhead camera...")
        self.wait_for_camera_frames(1)
        saved = 0
        attempts = 0
        while rclpy.ok() and saved < self.sample_count and attempts < self.sample_count * 3:
            attempts += 1
            positions = self.generate_positions()
            if not all(
                self.set_cube_pose(
                    self.MODEL_NAMES[color], x, y,
                    random.uniform(-math.pi, math.pi),
                )
                for color, (x, y) in positions.items()
            ):
                continue

            # Critical: drain stale queued frames after moving the cubes.
            self.wait_for_camera_frames(6)
            image = self.latest_image.copy()
            preview = image.copy()
            image_height, image_width = image.shape[:2]
            labels = []
            for color, class_id in self.CLASS_IDS.items():
                box = self.find_cube_box(image, color, *positions[color])
                if box is None:
                    self.get_logger().warning(f"{color} cube not visible")
                    continue
                labels.append(self.yolo_label(class_id, box, image_width, image_height))
                x1, y1, x2, y2 = box
                cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(preview, color, (x1, max(20, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            if not labels:
                continue
            split = "val" if random.random() < 0.20 else "train"
            name = f"cube_{int(time.time() * 1000)}"
            cv2.imwrite(str(self.output_directory / "images" / split / f"{name}.jpg"), image)
            (self.output_directory / "labels" / split / f"{name}.txt").write_text(
                "\n".join(labels) + "\n"
            )
            if saved < 10 or saved % 25 == 0:
                cv2.imwrite(str(self.output_directory / "previews" / f"{name}.jpg"), preview)
            saved += 1
            self.get_logger().info(
                f"Saved sample {saved}/{self.sample_count}: {split}, labels={len(labels)}"
            )
        if saved != self.sample_count:
            raise RuntimeError(f"Only generated {saved}/{self.sample_count} samples")
        self.get_logger().info(f"Dataset complete: {self.output_directory}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDatasetGenerator()
    try:
        node.collect()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Make executable and generate data:

```bash
chmod +x /root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/generate_yolo_dataset.py
python3 /root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/generate_yolo_dataset.py \
  --ros-args -p use_sim_time:=true -p samples:=10
python3 /root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/generate_yolo_dataset.py \
  --ros-args -p use_sim_time:=true -p samples:=300
```

Final dataset: 310 images, 310 label files, 310 instances per class, 243 train images, 67 validation images, and zero invalid labels.

## Train and validate

Stop Gazebo and MoveIt to free CPU, then run:

```bash
source /root/yolo_env/bin/activate
ASSETS=$(python3 -c "from mycobot_yolo_assets.paths import yolo_assets_dir; print(yolo_assets_dir())")
yolo detect train model=$ASSETS/yolo11n.pt data=$ASSETS/dataset/data.yaml \
  epochs=50 imgsz=416 batch=8 device=cpu workers=2 patience=15 \
  hsv_h=0.0 \
  project=/root/mycobot_ws/src/mycobot_yolo_assets/assets/runs \
  name=cube_detector plots=True
```

`hsv_h=0.0` is essential because changing hue would contradict color-based class labels.

```bash
yolo detect val \
  model=$(python3 -c "from mycobot_yolo_assets.paths import default_model_path; print(default_model_path())") \
  data=$(python3 -c "from mycobot_yolo_assets.paths import default_dataset_yaml; print(default_dataset_yaml())") \
  imgsz=416 batch=8 device=cpu plots=True
```

| Class | Precision | Recall | mAP50 | mAP50–95 |
| --- | ---: | ---: | ---: | ---: |
| All | 0.999 | 1.000 | 0.995 | 0.935 |
| Red | 0.999 | 1.000 | 0.995 | 0.942 |
| Green | 0.999 | 1.000 | 0.995 | 0.940 |
| Blue | 0.999 | 1.000 | 0.995 | 0.923 |

CPU inference was approximately 8.6 ms/image. `mAP50–95=0.935` is average precision over IoU thresholds 0.50–0.95; it is not detection confidence.

## Live YOLO ROS 2 detector

Save as `mycobot_sim_projects/mycobot_sim_projects/yolo_cube_detector.py`:

```python
#!/usr/bin/env python3
import os
import sys

# A ros2 console script may start with system Python. Re-execute under the
# virtual environment while preserving all ROS arguments if necessary.
try:
    from ultralytics import YOLO
except ModuleNotFoundError:
    venv_python = "/root/yolo_env/bin/python3"
    if os.path.exists(venv_python) and sys.executable != venv_python:
        os.execv(
            venv_python,
            [venv_python, os.path.abspath(__file__), *sys.argv[1:]],
        )
    raise

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloCubeDetector(Node):
    def __init__(self):
        super().__init__("yolo_cube_detector")
        self.declare_parameter(
            "model_path",
            "/root/yolo_runs/cube_detector/weights/best.pt",
        )
        self.declare_parameter("target_color", "red")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("image_size", 416)

        self.model_path = str(self.get_parameter("model_path").value)
        self.target_color = str(
            self.get_parameter("target_color").value
        ).lower()
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.image_size = int(self.get_parameter("image_size").value)

        if self.target_color not in ("red", "green", "blue"):
            raise ValueError("target_color must be red, green, or blue")
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"YOLO model does not exist: {self.model_path}"
            )

        self.target_class_name = f"{self.target_color}_cube"
        self.bridge = CvBridge()
        self.frame_counter = 0
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        self.pixel_publisher = self.create_publisher(
            PointStamped, "/selected_cube/pixel_center", 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, "/selected_cube/annotated_image", 10
        )
        self.create_subscription(
            Image,
            "/overhead_camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"YOLO detector ready. Target: {self.target_class_name}, "
            f"confidence threshold: {self.confidence_threshold:.2f}"
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            result = self.model.predict(
                source=image,
                imgsz=self.image_size,
                conf=self.confidence_threshold,
                device="cpu",
                verbose=False,
            )[0]
            annotated = result.plot()
            selected_box = None
            selected_confidence = -1.0

            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]
                if (
                    class_name == self.target_class_name
                    and confidence > selected_confidence
                ):
                    selected_box = box
                    selected_confidence = confidence

            self.frame_counter += 1
            if selected_box is not None:
                x1, y1, x2, y2 = selected_box.xyxy[0].tolist()
                centre_x = (x1 + x2) / 2.0
                centre_y = (y1 + y2) / 2.0

                point = PointStamped()
                point.header = message.header
                point.point.x = centre_x
                point.point.y = centre_y
                point.point.z = 0.0
                self.pixel_publisher.publish(point)

                cv2.circle(
                    annotated,
                    (int(round(centre_x)), int(round(centre_y))),
                    5, (0, 255, 255), -1,
                )
                cv2.putText(
                    annotated,
                    f"selected: {self.target_class_name} {selected_confidence:.2f}",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2, cv2.LINE_AA,
                )
                if self.frame_counter % 10 == 0:
                    self.get_logger().info(
                        f"{self.target_class_name}: "
                        f"confidence={selected_confidence:.3f}, "
                        f"pixel=({centre_x:.1f}, {centre_y:.1f})"
                    )
            else:
                cv2.putText(
                    annotated,
                    f"{self.target_class_name} not detected",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2, cv2.LINE_AA,
                )
                if self.frame_counter % 30 == 0:
                    self.get_logger().warning(
                        f"{self.target_class_name} not detected"
                    )

            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
            annotated_message.header = message.header
            self.annotated_publisher.publish(annotated_message)
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloCubeDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Make it executable and register it in `mycobot_sim_projects/setup.py`:

```bash
chmod +x /root/mycobot_ws/src/mycobot_sim_projects/mycobot_sim_projects/yolo_cube_detector.py
```

```python
entry_points={
    "console_scripts": [
        "color_cube_detector = mycobot_sim_projects.color_cube_detector:main",
        "pixel_to_world = mycobot_sim_projects.pixel_to_world:main",
        "yolo_cube_detector = mycobot_sim_projects.yolo_cube_detector:main",
    ],
},
```

Build and test:

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash
source /root/yolo_env/bin/activate
colcon build --symlink-install --packages-select mycobot_sim_projects
source install/setup.bash
ros2 run mycobot_sim_projects yolo_cube_detector \
  --ros-args -p use_sim_time:=true -p target_color:=red
```

Inspect the selected pixel and annotated image:

```bash
ros2 topic echo /selected_cube/pixel_center --once
ros2 run rqt_image_view rqt_image_view
```

Choose `/selected_cube/annotated_image` in `rqt_image_view`. The fresh Gazebo test detected red at 0.970 confidence, green at 0.957, and blue at 0.942.

## Complete `cube_approach.launch.py`

This launch supports YOLO by default and retains HSV as a fallback:

```python
"""Launch vision-guided cube pick-and-place.

Requires gazebo_sim.launch.py and gazebo_move_group.launch.py first.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _on_cube_exit(event, context):
    del context
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"cube_approach finished with exit code {event.returncode}"
            )
        )
    ]


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "mycobot_280jn_sim",
            package_name="mycobot_280jn_moveit_config",
        )
        .to_moveit_configs()
    )
    target_color = LaunchConfiguration("target_color")

    hsv_detector = Node(
        package="mycobot_sim_projects",
        executable="color_cube_detector",
        output="screen",
        parameters=[{"use_sim_time": True, "target_color": target_color}],
        condition=LaunchConfigurationEquals("detector", "hsv"),
    )
    yolo_detector = Node(
        package="mycobot_sim_projects",
        executable="yolo_cube_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "target_color": target_color,
            "model_path": LaunchConfiguration("model_path"),
            "confidence_threshold": ParameterValue(
                LaunchConfiguration("confidence_threshold"), value_type=float
            ),
            "image_size": ParameterValue(
                LaunchConfiguration("image_size"), value_type=int
            ),
        }],
        condition=LaunchConfigurationEquals("detector", "yolo"),
    )
    pixel_to_world = Node(
        package="mycobot_sim_projects",
        executable="pixel_to_world",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    cube_approach = Node(
        package="mycobot_moveit_projects",
        executable="cube_approach",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "place_x": ParameterValue(
                    LaunchConfiguration("place_x"), value_type=float
                ),
                "place_y": ParameterValue(
                    LaunchConfiguration("place_y"), value_type=float
                ),
                "place_z": ParameterValue(
                    LaunchConfiguration("place_z"), value_type=float
                ),
                "place_descend_z": ParameterValue(
                    LaunchConfiguration("place_descend_z"), value_type=float
                ),
                "return_home": ParameterValue(
                    LaunchConfiguration("return_home"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("detector", default_value="yolo"),
        DeclareLaunchArgument(
            "model_path",
            default_value="/root/yolo_runs/cube_detector/weights/best.pt",
        ),
        DeclareLaunchArgument("confidence_threshold", default_value="0.50"),
        DeclareLaunchArgument("image_size", default_value="416"),
        DeclareLaunchArgument("target_color", default_value="red"),
        DeclareLaunchArgument("place_x", default_value="0.10"),
        DeclareLaunchArgument("place_y", default_value="0.15"),
        DeclareLaunchArgument("place_z", default_value="0.140"),
        DeclareLaunchArgument("place_descend_z", default_value="0.055"),
        DeclareLaunchArgument("return_home", default_value="true"),
        hsv_detector,
        yolo_detector,
        pixel_to_world,
        TimerAction(period=2.0, actions=[cube_approach]),
        RegisterEventHandler(
            OnProcessExit(target_action=cube_approach, on_exit=_on_cube_exit)
        ),
    ])
```

Important: default weights resolve via `mycobot_yolo_assets.paths.default_model_path()` — typically `.../share/mycobot_yolo_assets/assets/runs/cube_detector/weights/best.pt` after build, or `src/mycobot_yolo_assets/assets/...` in source tree.

## Build the complete system

```bash
cd /root/mycobot_ws
source /opt/ros/humble/setup.bash
source /root/yolo_env/bin/activate
colcon build --symlink-install --packages-select \
  mycobot_yolo_assets mycobot_sim_projects mycobot_moveit_projects
source install/setup.bash
python3 -c "from mycobot_yolo_assets.paths import default_model_path; print(default_model_path())"
```

## Essential runtime commands

Use three terminals inside the running container.

### Terminal 1 — Gazebo

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 launch mycobot_280jn_sim gazebo_sim.launch.py
```

### Terminal 2 — MoveIt

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
ros2 launch mycobot_280jn_moveit_config gazebo_move_group.launch.py
```

### Terminal 3 — YOLO pick-and-place

```bash
source /opt/ros/humble/setup.bash
source /root/mycobot_ws/install/setup.bash
source /root/yolo_env/bin/activate
```

Red cube:

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  detector:=yolo target_color:=red place_x:=0.10 place_y:=0.20
```

Green cube:

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  detector:=yolo target_color:=green place_x:=0.00 place_y:=0.20
```

Blue cube:

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  detector:=yolo target_color:=blue place_x:=-0.10 place_y:=0.20
```

YOLO is the default, so `detector:=yolo` can be omitted. To use the earlier fallback:

```bash
ros2 launch mycobot_moveit_projects cube_approach.launch.py \
  detector:=hsv target_color:=red
```

## Important diagnostic commands

```bash
# Enter or open another container terminal
docker start -ai mycobot-humble
docker exec -it mycobot-humble bash

# Camera and bridge
ign topic -l | grep -Ei "overhead|camera|image"
ros2 node list | grep overhead_camera_bridge
ros2 topic info /overhead_camera/image
ros2 topic echo /overhead_camera/camera_info --once

# Perception pipeline
ros2 topic echo /selected_cube/pixel_center --once
ros2 topic echo /selected_cube/world_center --once
ros2 topic info /selected_cube/annotated_image

# Robot state and MoveIt
ros2 topic echo /joint_states --once
ros2 action list | grep move_group

# Model files
find $(ros2 pkg prefix mycobot_yolo_assets)/share/mycobot_yolo_assets/assets/runs \
  -type f \( -name "best.pt" -o -name "last.pt" \) -exec ls -lh {} \;

# Launch parameters
ros2 launch mycobot_moveit_projects cube_approach.launch.py --show-args
```

## Main problems and solutions

| Problem | Cause | Solution |
| --- | --- | --- |
| Camera image missing | Gazebo or bridge not running | Start `gazebo_sim.launch.py`; check `/overhead_camera/image` |
| Cube hidden during grasp | Camera directly above robot | Use pose `0.30 -0.25 1.30 0 1.5708 0` |
| Dataset labels missing | Old camera frames remained queued after cube movement | Wait for six fresh callbacks before saving |
| `torch`/`ultralytics` missing | Packages absent from system Python | Use `/root/yolo_env` with system site packages |
| `cv_bridge` unavailable in venv | Isolated venv cannot see ROS packages | Create venv using `--system-site-packages` |
| Package hash mismatch | Corrupt cached download | Purge cache and retry with `--no-cache-dir`; never bypass checking |
| CUDA unavailable | Container has no NVIDIA runtime | Use `device=cpu`; YOLO11n is fast enough |
| YOLO launch cannot find weights | Package not built | `colcon build --symlink-install --packages-select mycobot_yolo_assets`; source install |
| Annotated image works but robot does not move | Downstream converter, MoveIt, or controllers missing | Check pixel topic, world topic, `move_group`, and joint states |
| Wrong cube selected | Incorrect `target_color` | Use only `red`, `green`, or `blue` |

When splitting Bash commands, `\` must be the final character on its line; do not add a trailing space.

## What changed in this project

1. Added PyTorch and Ultralytics in a ROS-compatible virtual environment.
2. Added programmatic Gazebo cube randomization.
3. Added an automatic YOLO dataset generator.
4. Corrected stale-frame synchronization by draining six camera frames.
5. Generated a balanced dataset with 310 images and 930 instances.
6. Trained YOLO11n for 50 epochs on CPU.
7. Validated the trained model quantitatively and on a fresh Gazebo frame.
8. Added `yolo_cube_detector.py` as a live ROS 2 inference node.
9. Kept the same `/selected_cube/pixel_center` contract as the HSV node.
10. Updated `cube_approach.launch.py` to select YOLO or HSV.
11. Made YOLO the default while retaining HSV as a fallback.
12. Reused the validated pixel-to-world and MoveIt manipulation pipeline.

## Limitations and next steps

The high metrics apply to the simulated domain. Before using the physical robot:

1. Collect real webcam images under varied lighting.
2. Add real cube, gripper, table, shadow, and occlusion examples.
3. Add negative images with no cubes.
4. Randomize Gazebo lighting, textures, noise, camera position, and backgrounds.
5. Calibrate the real camera and use TF-based geometric projection.
6. Require several consistent detections before triggering motion.
7. Package `best.pt` in `mycobot_yolo_assets` (committed under `src/`) instead of ad-hoc `/root/yolo_runs` paths.
8. Attach the grasped cube to the MoveIt planning scene during transport.

## Completion checklist

- [x] Shifted overhead camera prevents the original severe occlusion.
- [x] Gazebo image and camera-info topics are bridged to ROS 2.
- [x] Red, green, and blue Gazebo cubes can be randomized.
- [x] 310 valid images and 930 balanced object labels generated.
- [x] YOLO11n trained successfully on CPU.
- [x] Overall mAP50 `0.995` and mAP50–95 `0.935`.
- [x] Fresh Gazebo frame detects all three cube classes.
- [x] Live ROS 2 node publishes the selected pixel centre.
- [x] Annotated YOLO stream works in `rqt_image_view`.
- [x] Launch file supports YOLO and HSV.
- [ ] Record final end-to-end YOLO-guided pick for all three colors.
- [ ] Add real-camera data for sim-to-real transfer.

## Final outcome

The project successfully moves from deterministic OpenCV color segmentation to learned multi-class object detection. YOLO recognizes all three cubes and plugs into the same ROS perception interface, so the existing coordinate conversion and MoveIt manipulation code remain modular and reusable.
