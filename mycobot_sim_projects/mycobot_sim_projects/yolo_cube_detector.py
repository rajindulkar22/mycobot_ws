#!/usr/bin/env python3

import os
import sys

# ros2 run may use the system Python. Re-launch this node with the YOLO
# virtual environment when Ultralytics is unavailable.
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
from mycobot_yolo_assets.paths import default_model_path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloCubeDetector(Node):

    def __init__(self):
        super().__init__("yolo_cube_detector")

        self.declare_parameter(
            "model_path",
            default_model_path(),
        )
        self.declare_parameter("target_color", "red")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("image_size", 416)

        self.model_path = str(
            self.get_parameter("model_path").value
        )
        self.target_color = str(
            self.get_parameter("target_color").value
        ).lower()
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.image_size = int(
            self.get_parameter("image_size").value
        )

        if self.target_color not in ("red", "green", "blue"):
            raise ValueError(
                "target_color must be red, green, or blue"
            )

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"YOLO model does not exist: {self.model_path}"
            )

        self.target_class_name = f"{self.target_color}_cube"
        self.bridge = CvBridge()
        self.frame_counter = 0

        self.get_logger().info(
            f"Loading YOLO model: {self.model_path}"
        )
        self.model = YOLO(self.model_path)

        self.pixel_publisher = self.create_publisher(
            PointStamped,
            "/selected_cube/pixel_center",
            10,
        )

        self.annotated_publisher = self.create_publisher(
            Image,
            "/selected_cube/annotated_image",
            10,
        )

        self.image_subscription = self.create_subscription(
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
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

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
                    5,
                    (0, 255, 255),
                    -1,
                )

                cv2.putText(
                    annotated,
                    (
                        f"selected: {self.target_class_name} "
                        f"{selected_confidence:.2f}"
                    ),
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                self.frame_counter += 1

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
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                self.frame_counter += 1

                if self.frame_counter % 30 == 0:
                    self.get_logger().warning(
                        f"{self.target_class_name} not detected"
                    )

            annotated_message = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8",
            )
            annotated_message.header = message.header
            self.annotated_publisher.publish(annotated_message)

        except Exception as error:
            self.get_logger().error(
                f"YOLO inference failed: {error}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = YoloCubeDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"yolo_cube_detector: {error}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()