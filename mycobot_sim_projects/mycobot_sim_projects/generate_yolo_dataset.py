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
from mycobot_yolo_assets.paths import default_dataset_dir
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDatasetGenerator(Node):

    CLASS_IDS = {
        "red": 0,
        "green": 1,
        "blue": 2,
    }

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
        "green": [
            (np.array([35, 70, 40]), np.array([90, 255, 255])),
        ],
        "blue": [
            (np.array([90, 70, 40]), np.array([140, 255, 255])),
        ],
    }

    def __init__(self):
        super().__init__("generate_yolo_dataset")

        self.declare_parameter("samples", 10)
        self.declare_parameter(
            "output_directory",
            default_dataset_dir(),
        )

        self.sample_count = int(
            self.get_parameter("samples").value
        )
        self.output_directory = Path(
            self.get_parameter("output_directory").value
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.frame_number = 0

        self.subscription = self.create_subscription(
            Image,
            "/overhead_camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Gazebo camera parameters.
        self.camera_x = 0.30
        self.camera_y = -0.25
        self.camera_z = 1.30
        self.object_z = 0.4125

        self.fx = 554.2546463012695
        self.fy = 554.2547035217285
        self.cx = 320.0
        self.cy = 240.0

        self.kernel = np.ones((3, 3), np.uint8)

        self.prepare_directories()

    def prepare_directories(self):
        directories = [
            "images/train",
            "images/val",
            "labels/train",
            "labels/val",
            "previews",
        ]

        for directory in directories:
            (self.output_directory / directory).mkdir(
                parents=True,
                exist_ok=True,
            )

        yaml_content = f"""path: {self.output_directory}
train: images/train
val: images/val

names:
  0: red_cube
  1: green_cube
  2: blue_cube
"""

        (self.output_directory / "data.yaml").write_text(
            yaml_content
        )

    def image_callback(self, message):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            self.frame_number += 1
        except Exception as error:
            self.get_logger().error(
                f"Image conversion failed: {error}"
            )

    def set_cube_pose(self, model_name, x, y, yaw):
        quaternion_z = math.sin(yaw / 2.0)
        quaternion_w = math.cos(yaw / 2.0)

        request = (
            f'name: "{model_name}" '
            f"position {{ "
            f"x: {x:.6f} "
            f"y: {y:.6f} "
            f"z: {self.object_z:.6f} "
            f"}} "
            f"orientation {{ "
            f"z: {quaternion_z:.6f} "
            f"w: {quaternion_w:.6f} "
            f"}}"
        )

        command = [
            "ign",
            "service",
            "-s",
            "/world/mycobot_world/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        ]

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )

        if result.returncode != 0 or "data: true" not in result.stdout:
            self.get_logger().error(
                f"Failed to move {model_name}: "
                f"{result.stdout} {result.stderr}"
            )
            return False

        return True

    def generate_positions(self):
        positions = {}

        for color in self.CLASS_IDS:
            for _ in range(100):
                x = random.uniform(0.08, 0.28)
                y = random.uniform(-0.18, 0.18)

                sufficiently_separated = all(
                    math.hypot(
                        x - previous_x,
                        y - previous_y,
                    ) >= 0.065
                    for previous_x, previous_y in positions.values()
                )

                if sufficiently_separated:
                    positions[color] = (x, y)
                    break
            else:
                raise RuntimeError(
                    "Could not generate separated cube positions."
                )

        return positions

    def projected_pixel(self, world_x, world_y):
        depth = self.camera_z - self.object_z

        pixel_u = self.cx - (
            (world_y - self.camera_y) * self.fx / depth
        )
        pixel_v = self.cy - (
            (world_x - self.camera_x) * self.fy / depth
        )

        return pixel_u, pixel_v

    def create_mask(self, hsv_image, color):
        mask = np.zeros(
            hsv_image.shape[:2],
            dtype=np.uint8,
        )

        for lower, upper in self.HSV_RANGES[color]:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv_image, lower, upper),
            )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.kernel,
        )

        return mask

    def find_cube_box(
        self,
        image,
        color,
        expected_x,
        expected_y,
    ):
        hsv_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )
        mask = self.create_mask(hsv_image, color)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        expected_u, expected_v = self.projected_pixel(
            expected_x,
            expected_y,
        )

        best_contour = None
        best_distance = float("inf")

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 30:
                continue

            moments = cv2.moments(contour)

            if moments["m00"] == 0:
                continue

            centre_u = moments["m10"] / moments["m00"]
            centre_v = moments["m01"] / moments["m00"]

            distance = math.hypot(
                centre_u - expected_u,
                centre_v - expected_v,
            )

            if distance < best_distance:
                best_distance = distance
                best_contour = contour

        if best_contour is None or best_distance > 50:
            return None

        x, y, width, height = cv2.boundingRect(best_contour)

        padding = 2
        image_height, image_width = image.shape[:2]

        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(image_width - 1, x + width + padding)
        y2 = min(image_height - 1, y + height + padding)

        return x1, y1, x2, y2

    @staticmethod
    def yolo_label(class_id, box, image_width, image_height):
        x1, y1, x2, y2 = box

        centre_x = ((x1 + x2) / 2.0) / image_width
        centre_y = ((y1 + y2) / 2.0) / image_height
        width = (x2 - x1) / image_width
        height = (y2 - y1) / image_height

        return (
            f"{class_id} "
            f"{centre_x:.6f} "
            f"{centre_y:.6f} "
            f"{width:.6f} "
            f"{height:.6f}"
        )

    def wait_for_camera_frames(self, number_of_frames=1):
        target_frame = self.frame_number + number_of_frames
        deadline = time.monotonic() + 120.0

        while rclpy.ok() and self.frame_number < target_frame:
            rclpy.spin_once(self, timeout_sec=0.2)

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Timed out waiting for fresh camera frames."
                )

    def collect(self):
        self.get_logger().info(
            "Waiting for the overhead camera..."
        )

        self.wait_for_camera_frames(1)

        saved_samples = 0
        attempts = 0
        maximum_attempts = self.sample_count * 3

        while (
            rclpy.ok()
            and saved_samples < self.sample_count
            and attempts < maximum_attempts
        ):
            attempts += 1
            positions = self.generate_positions()

            movement_succeeded = True

            for color, (x, y) in positions.items():
                yaw = random.uniform(-math.pi, math.pi)

                if not self.set_cube_pose(
                    self.MODEL_NAMES[color],
                    x,
                    y,
                    yaw,
                ):
                    movement_succeeded = False
                    break

            if not movement_succeeded:
                continue

            # Drain previously queued images and allow Gazebo to render the new poses.
            self.wait_for_camera_frames(6)

            image = self.latest_image.copy()
            preview = image.copy()
            image_height, image_width = image.shape[:2]
            labels = []

            for color, class_id in self.CLASS_IDS.items():
                world_x, world_y = positions[color]

                box = self.find_cube_box(
                    image,
                    color,
                    world_x,
                    world_y,
                )

                if box is None:
                    self.get_logger().warning(
                        f"{color} cube was not visible; "
                        "saving labels for visible cubes only."
                    )
                    continue

                labels.append(
                    self.yolo_label(
                        class_id,
                        box,
                        image_width,
                        image_height,
                    )
                )

                x1, y1, x2, y2 = box
                cv2.rectangle(
                    preview,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    preview,
                    color,
                    (x1, max(20, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            # Do not use frames where every cube was missed.
            if not labels:
                continue

            split = "val" if random.random() < 0.20 else "train"
            filename = f"cube_{int(time.time() * 1000)}"

            image_path = (
                self.output_directory
                / "images"
                / split
                / f"{filename}.jpg"
            )
            label_path = (
                self.output_directory
                / "labels"
                / split
                / f"{filename}.txt"
            )

            cv2.imwrite(str(image_path), image)
            label_path.write_text("\n".join(labels) + "\n")

            if saved_samples < 10 or saved_samples % 25 == 0:
                preview_path = (
                    self.output_directory
                    / "previews"
                    / f"{filename}.jpg"
                )
                cv2.imwrite(str(preview_path), preview)

            saved_samples += 1

            self.get_logger().info(
                f"Saved sample {saved_samples}/{self.sample_count}: "
                f"{split}, labels={len(labels)}"
            )

        if saved_samples != self.sample_count:
            raise RuntimeError(
                f"Only generated {saved_samples} of "
                f"{self.sample_count} requested samples."
            )

        self.get_logger().info(
            f"Dataset complete: {self.output_directory}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDatasetGenerator()

    try:
        node.collect()
    except KeyboardInterrupt:
        node.get_logger().info("Dataset generation interrupted.")
    except Exception as error:
        node.get_logger().error(str(error))
        raise
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()