#!/usr/bin/env python3
"""Detect a selected red, green or blue cube."""

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# HSV colour thresholds per cube (OpenCV hue is 0–179, S/V are 0–255).
# Each colour maps to a list of (lower, upper) bounds; red needs two ranges
# because hue wraps at 0/180.
COLOR_RANGES = {
    "red": [
        (
            np.array([0, 120, 70], dtype=np.uint8),    # low-red hue band
            np.array([10, 255, 255], dtype=np.uint8),
        ),
        (
            np.array([170, 120, 70], dtype=np.uint8),  # high-red hue band
            np.array([179, 255, 255], dtype=np.uint8),
        ),
    ],
    "green": [
        (
            np.array([35, 100, 60], dtype=np.uint8),
            np.array([85, 255, 255], dtype=np.uint8),
        ),
    ],
    "blue": [
        (
            np.array([95, 100, 60], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8),
        ),
    ],
}


# BGR colours for drawing debug overlays (OpenCV uses BGR, not RGB).
BOX_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
}


class ColorCubeDetector(Node):
    """Find the largest blob of target_color and publish its pixel centre."""

    def __init__(self) -> None:
        super().__init__("color_cube_detector")

        # Camera topic from Gazebo (overhead_camera in mycobot_table.sdf).
        self.declare_parameter(
            "image_topic",
            "/overhead_camera/image",
        )
        # Which cube to track: red, green, or blue.
        self.declare_parameter("target_color", "red")
        # Ignore tiny blobs (sensor noise, reflections).
        self.declare_parameter("minimum_area", 100.0)

        image_topic = self.get_parameter("image_topic").value
        self.target_color = (
            str(self.get_parameter("target_color").value).lower()
        )
        self.minimum_area = float(
            self.get_parameter("minimum_area").value
        )

        if self.target_color not in COLOR_RANGES:
            raise ValueError(
                "target_color must be red, green or blue"
            )

        # Converts between sensor_msgs/Image and OpenCV numpy arrays.
        self.bridge = CvBridge()
        self.frame_count = 0

        # Best-effort QoS matches typical camera publishers (low latency).
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Debug view: same frame with bounding box and centre drawn on it.
        self.annotated_publisher = self.create_publisher(
            Image,
            "/selected_cube/annotated_image",
            10,
        )

        # Cube centre in image pixels: point.x = column, point.y = row.
        self.center_publisher = self.create_publisher(
            PointStamped,
            "/selected_cube/pixel_center",
            10,
        )

        self.get_logger().info(
            f"Target colour: {self.target_color}"
        )
        self.get_logger().info(
            f"Listening on {image_topic}"
        )

    def create_color_mask(
        self,
        hsv_image: np.ndarray,
    ) -> np.ndarray:
        """Build a binary mask of pixels matching the selected cube colour."""

        # Start with all-black mask (same height/width as image, single channel).
        mask = np.zeros(
            hsv_image.shape[:2],
            dtype=np.uint8,
        )

        # Combine all HSV ranges for this colour (red uses two, OR'd together).
        for lower, upper in COLOR_RANGES[self.target_color]:
            current_mask = cv2.inRange(
                hsv_image,
                lower,
                upper,
            )
            mask = cv2.bitwise_or(mask, current_mask)

        # 5×5 structuring element for morphology cleanup.
        kernel = np.ones((5, 5), dtype=np.uint8)

        # OPEN  → erode then dilate: removes isolated noise pixels.
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )
        # CLOSE → dilate then erode: fills small holes inside the cube blob.
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        return mask

    def image_callback(self, message: Image) -> None:
        """Detect target-colour pixels in one camera frame and publish results."""

        # --- Step 1: ROS Image → OpenCV BGR array ---
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Image conversion failed: {error}"
            )
            return

        # --- Step 2: BGR → HSV ---
        # HSV separates hue (colour) from brightness, so colour detection is
        # more stable under shadows and lighting changes than raw BGR thresholds.
        hsv_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )

        # --- Step 3: Build a binary mask of target-colour pixels ---
        mask = self.create_color_mask(hsv_image)

        # --- Step 4: Find connected colour regions (contours) ---
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,       # only outer boundaries, not nested holes
            cv2.CHAIN_APPROX_SIMPLE,  # compress contour points
        )

        detected = False

        if contours:
            # Assume the pick target is the largest blob of this colour in frame.
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            if area >= self.minimum_area:
                # Axis-aligned bounding box around the contour.
                x, y, width, height = cv2.boundingRect(
                    contour
                )

                # Image origin is top-left; x increases right, y increases down.
                center_x = x + width / 2.0
                center_y = y + height / 2.0

                display_color = BOX_COLORS[
                    self.target_color
                ]

                # --- Step 5: Draw debug overlay on the output image ---
                cv2.rectangle(
                    image,
                    (x, y),
                    (x + width, y + height),
                    display_color,
                    2,
                )

                cv2.circle(
                    image,
                    (int(center_x), int(center_y)),
                    5,
                    (255, 255, 255),  # white dot at centre
                    -1,
                )

                label = (
                    f"{self.target_color} cube: "
                    f"({center_x:.1f}, {center_y:.1f})"
                )

                cv2.putText(
                    image,
                    label,
                    (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    display_color,
                    2,
                )

                # --- Step 6: Publish pixel centre (2D only, z unused) ---
                center_message = PointStamped()
                center_message.header = message.header  # same timestamp as camera
                center_message.point.x = center_x
                center_message.point.y = center_y
                center_message.point.z = 0.0

                self.center_publisher.publish(
                    center_message
                )

                self.frame_count += 1
                detected = True

                # Log every 30 detections (~3 s at 10 Hz camera) to avoid spam.
                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        f"{self.target_color} cube: "
                        f"pixel=({center_x:.1f}, "
                        f"{center_y:.1f}), "
                        f"area={area:.1f}"
                    )

        if not detected:
            cv2.putText(
                image,
                f"{self.target_color} cube not detected",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        # Always publish the annotated frame (detected or not) for debugging.
        annotated = self.bridge.cv2_to_imgmsg(
            image,
            encoding="bgr8",
        )
        annotated.header = message.header

        self.annotated_publisher.publish(annotated)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorCubeDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
