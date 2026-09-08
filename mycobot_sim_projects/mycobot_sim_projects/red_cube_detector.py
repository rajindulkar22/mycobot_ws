#!/usr/bin/env python3
"""Detect the red cube in the overhead Gazebo camera image.

Pipeline (runs on every camera frame):
  /overhead_camera/image  →  HSV colour mask  →  largest red blob
                           →  /red_cube/pixel_center      (2D centre in pixels)
                           →  /red_cube/annotated_image   (debug overlay)

The overhead camera is defined in mycobot_table.sdf and bridged to ROS by
gazebo_sim.launch.py. This node does NOT convert pixels to 3D table coords;
it only reports where the cube appears in the image.
"""

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RedCubeDetector(Node):
    """Find the largest red object and publish its pixel centre."""

    def __init__(self) -> None:
        super().__init__("red_cube_detector")

        # Camera topic from Gazebo (overhead_camera in mycobot_table.sdf).
        self.declare_parameter(
            "image_topic",
            "/overhead_camera/image",
        )
        # Ignore tiny red blobs (sensor noise, reflections).
        self.declare_parameter(
            "minimum_area",
            100.0,
        )

        image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        self.minimum_area = (
            self.get_parameter("minimum_area")
            .get_parameter_value()
            .double_value
        )

        # Converts between sensor_msgs/Image and OpenCV numpy arrays.
        self.bridge = CvBridge()
        self.frame_count = 0

        # Best-effort QoS matches typical camera publishers (low latency).
        self.image_subscriber = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Debug view: same frame with bounding box and centre drawn on it.
        self.annotated_publisher = self.create_publisher(
            Image,
            "/red_cube/annotated_image",
            10,
        )

        # Cube centre in image pixels: point.x = column, point.y = row.
        self.centre_publisher = self.create_publisher(
            PointStamped,
            "/red_cube/pixel_center",
            10,
        )

        self.get_logger().info(
            f"Listening for camera images on {image_topic}"
        )

    def image_callback(self, message: Image) -> None:
        """Detect red pixels in one camera frame and publish results."""

        # --- Step 1: ROS Image → OpenCV BGR array ---
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Could not convert camera image: {error}"
            )
            return

        # --- Step 2: BGR → HSV ---
        # HSV separates hue (colour) from brightness, so red detection is
        # more stable under shadows and lighting changes than raw BGR thresholds.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # --- Step 3: Build a binary mask of "red" pixels ---
        # Hue wraps at 0/180, so red appears near both ends of the scale.
        #   H (0–179):   0–10 and 170–179  → red hues
        #   S (0–255):   ≥120              → ignore washed-out / gray pixels
        #   V (0–255):   ≥70               → ignore very dark pixels
        lower_red_1 = np.array([0, 120, 70], dtype=np.uint8) # red 1
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8) # red 1

        lower_red_2 = np.array([170, 120, 70], dtype=np.uint8) # red 2
        upper_red_2 = np.array([179, 255, 255], dtype=np.uint8) # red 2

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1) # red 1
        mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2) # red 2
        mask = cv2.bitwise_or(mask_1, mask_2) # red 1 or red 2

        # --- Step 4: Clean up the mask ---
        # OPEN  → erode then dilate: removes isolated noise pixels.
        # CLOSE → dilate then erode: fills small holes inside the cube blob.
        kernel = np.ones((5, 5), dtype=np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        # --- Step 5: Find connected red regions (contours) ---
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,       # only outer boundaries, not nested holes
            cv2.CHAIN_APPROX_SIMPLE,  # compress contour points
        )

        detected = False

        if contours:
            # Assume the pick target is the largest red blob in the frame.
            largest_contour = max(
                contours,
                key=cv2.contourArea,
            )

            area = cv2.contourArea(largest_contour)

            if area >= self.minimum_area:
                # Axis-aligned bounding box around the contour.
                x, y, width, height = cv2.boundingRect(
                    largest_contour
                )

                # Image origin is top-left; x increases right, y increases down.
                centre_x = x + width / 2.0
                centre_y = y + height / 2.0

                # --- Step 6: Draw debug overlay on the output image ---
                cv2.rectangle(
                    image,
                    (x, y),
                    (x + width, y + height),
                    (0, 255, 0),  # green box
                    2,
                )

                cv2.circle(
                    image,
                    (int(centre_x), int(centre_y)),
                    5,
                    (255, 0, 0),  # blue dot at centre
                    -1,
                )

                label = (
                    f"red cube: "
                    f"({centre_x:.1f}, {centre_y:.1f})"
                )

                cv2.putText(
                    image,
                    label,
                    (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                # --- Step 7: Publish pixel centre (2D only, z unused) ---
                centre_message = PointStamped()
                centre_message.header = message.header  # same timestamp as camera
                centre_message.point.x = centre_x
                centre_message.point.y = centre_y
                centre_message.point.z = 0.0

                self.centre_publisher.publish(centre_message)
                detected = True

                self.frame_count += 1

                # Log every 30 detections (~3 s at 10 Hz camera) to avoid spam.
                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        "Red cube detected: "
                        f"centre=({centre_x:.1f}, {centre_y:.1f}) px, "
                        f"area={area:.1f} px²"
                    )

        if not detected:
            cv2.putText(
                image,
                "Red cube not detected",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        # Always publish the annotated frame (detected or not) for debugging.
        annotated_message = self.bridge.cv2_to_imgmsg(
            image,
            encoding="bgr8",
        )
        annotated_message.header = message.header

        self.annotated_publisher.publish(annotated_message)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RedCubeDetector()

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
