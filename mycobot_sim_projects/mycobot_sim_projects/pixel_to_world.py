#!/usr/bin/env python3
"""Convert overhead-camera cube pixels into Gazebo world coordinates.

Pipeline (runs after red_cube_detector):
  /red_cube/pixel_center  +  /overhead_camera/camera_info
      →  /red_cube/world_center   (cube centre in metres, frame_id = "world")

Why this node exists:
  red_cube_detector reports WHERE the cube is in the image (pixels).
  MoveIt and the arm need WHERE the cube is on the table (metres).
  This node performs that conversion using a fixed downward-facing camera.

Assumptions (see mycobot_table.sdf overhead_camera pose):
  - Camera is static at (0.30, -0.25, 1.30) in Gazebo world.
  - MoveIt world frame is 0.405 m below Gazebo Z: camera at (0.30, -0.25, 0.895).
  - Camera pitch = 90° (looks straight down at the table).
  - Cube centre height in MoveIt world: object_z = 0.0075 m.
"""

import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class PixelToWorld(Node):
    """Map 2D pixel detections to 3D points on the table plane."""

    def __init__(self) -> None:
        super().__init__("pixel_to_world")

        # Must match overhead_camera pose in mycobot_table.sdf: 0.30 -0.25 1.30 0 1.5708 0
        # Camera and cube heights relative to the robot/MoveIt world frame.
        self.declare_parameter("camera_x", 0.30)
        self.declare_parameter("camera_y", -0.25)
        self.declare_parameter("camera_z", 0.895)
        self.declare_parameter("object_z", 0.0075)


        self.camera_x = self.get_parameter("camera_x").value # get the value of the parameter camera_x
        self.camera_y = self.get_parameter("camera_y").value # get the value of the parameter camera_y
        self.camera_z = self.get_parameter("camera_z").value # get the value of the parameter camera_z
        self.object_z = self.get_parameter("object_z").value # get the value of the parameter object_z

        # Pinhole intrinsics from CameraInfo.k (filled in camera_info_callback).
        #   K = [ fx   0  cx ]
        #       [  0  fy  cy ]
        #       [  0   0   1 ]
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.last_log_time = 0.0

        # Focal length and optical centre — needed for pixel → metre scaling.
        self.create_subscription(
            CameraInfo, # camera info message is a message that contains the camera intrinsic and extrinsic parameters
            "/overhead_camera/camera_info",
            self.camera_info_callback, # callback function for the camera info message
            qos_profile_sensor_data,
        )

        # Pixel centre from red_cube_detector (point.x = u, point.y = v).
        self.create_subscription(
            PointStamped, # point stamped message is a 3D point with a timestamp and a frame_id
            "/selected_cube/pixel_center",
            self.pixel_callback, # callback function for the point stamped message
            10,
        )

        # Cube location in Gazebo world frame — input for vision-guided picking.
        self.world_publisher = self.create_publisher(
            PointStamped,
            "/selected_cube/world_center",
            10,
        )

        self.get_logger().info(
            "Waiting for camera information and cube pixels..."
        )

    def camera_info_callback(
        self,
        message: CameraInfo,
    ) -> None:
        """Cache camera intrinsics from the bridged Gazebo camera."""

        self.fx = message.k[0]
        self.fy = message.k[4]
        self.cx = message.k[2]
        self.cy = message.k[5]

    def pixel_callback(
        self,
        message: PointStamped,
    ) -> None:
        """Convert one pixel detection to a world-frame table point."""

        # Cannot convert until camera_info has arrived at least once.
        if self.fx is None:
            return

        # Pixel coordinates from red_cube_detector (image row/column).
        u = message.point.x
        v = message.point.y

        # Vertical distance from camera to table plane (metres).
        depth = self.camera_z - self.object_z

        # Pinhole projection inverted for a downward-facing camera:
        #   pixel offset from image centre  →  offset on table  →  add camera position
        #
        # (cy - v): row increases downward in the image; world +X is "forward"
        #            on the table relative to this camera mount.
        # (u - cx): column increases rightward; world +Y uses opposite sign.
        world_x = (
            self.camera_x
            + (self.cy - v) * depth / self.fy
        )

        world_y = (
            self.camera_y
            - (u - self.cx) * depth / self.fx
        )

        # PointStamped carries frame + timestamp so MoveIt/TF know what this means.
        result = PointStamped()
        result.header.stamp = message.header.stamp  # same time as the detection
        result.header.frame_id = "world"

        result.point.x = world_x
        result.point.y = world_y
        result.point.z = self.object_z  # cube sits on the table at known height

        self.world_publisher.publish(result)

        # Log at most once per second to avoid flooding the console.
        now = time.monotonic()

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Pixel ({u:.1f}, {v:.1f}) -> "
                f"world ({world_x:.4f}, "
                f"{world_y:.4f}, {self.object_z:.4f}) m"
            )
            self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixelToWorld()

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
