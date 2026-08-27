#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException ,TransformListener  #  stores incoming transforms in a time-indexed tree so you can query "frame A relative to frame B at time T."
# transform listener subscribes to /tf and /tf_static and feeds every incoming message into a Buffer automatically.
''' TF messages arrive continuously and contain transformations at different timestamps.
 The buffer stores recent transforms and connects frame relationships'''



class TFExplorerNode(Node):
    """Report the myCobot gripper pose relative to the robot base."""

    def __init__(self):
        super().__init__("tf_explorer")

        self.base_frame = "joint1"
        self.tool_frame = "gripper_base"

        '''Two instance attributes holding the TF frame names to compare. 
        base_frame is the reference frame, tool_frame is the 
        frame whose pose you want expressed in base_frame's coordinates.'''

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.timer = self.create_timer(
            1.0,
            self.report_tool_pose,
        )

        self.get_logger().info(
            f"Monitoring transform: "
            f"{self.base_frame} → {self.tool_frame}"
        )

    def report_tool_pose(self):
        """Look up and print the latest gripper transform."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                Time(),
            )

        except TransformException as error:
            self.get_logger().warning(
                f"Transform unavailable: {error}"
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        roll, pitch, yaw = self.quaternion_to_euler(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        distance = math.sqrt(
            translation.x ** 2
            + translation.y ** 2
            + translation.z ** 2
        )

        horizontal_distance = math.sqrt(
            translation.x ** 2
            + translation.y ** 2
        )

        print("\nEnd-effector pose")
        print("-----------------")

        print(
            "Position [m]: "
            f"x={translation.x:.3f}, "
            f"y={translation.y:.3f}, "
            f"z={translation.z:.3f}"
        )

        print(
            "Quaternion [xyzw]: "
            f"[{rotation.x:.3f}, "
            f"{rotation.y:.3f}, "
            f"{rotation.z:.3f}, "
            f"{rotation.w:.3f}]"
        )

        print(
            "RPY [degrees]: "
            f"roll={math.degrees(roll):.1f}, "
            f"pitch={math.degrees(pitch):.1f}, "
            f"yaw={math.degrees(yaw):.1f}"
        )

        print(
            f"Distance from base: {distance:.3f} m"
        )

        print(
            f"Horizontal reach: {horizontal_distance:.3f} m"
        )

        print(
            f"Height: {translation.z:.3f} m"
        )
    '''
    Quaternion-to-Euler conversion

    The TF orientation is received as:

    q=(x,y,z,w)

    Our function converts it into:

    (ϕ,θ,ψ)=(roll,pitch,yaw)

    Quaternions are preferred internally, but Euler angles are easier for humans to interpret'''

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert a quaternion into roll, pitch and yaw."""

        sin_roll_cos_pitch = 2.0 * (
            w * x + y * z
        )

        cos_roll_cos_pitch = 1.0 - 2.0 * (
            x * x + y * y
        )

        roll = math.atan2(
            sin_roll_cos_pitch,
            cos_roll_cos_pitch,
        )

        sin_pitch = 2.0 * (
            w * y - z * x
        )

        sin_pitch = max(
            -1.0,
            min(1.0, sin_pitch),
        )

        pitch = math.asin(sin_pitch)

        sin_yaw_cos_pitch = 2.0 * (
            w * z + x * y
        )

        cos_yaw_cos_pitch = 1.0 - 2.0 * (
            y * y + z * z
        )

        yaw = math.atan2(
            sin_yaw_cos_pitch,
            cos_yaw_cos_pitch,
        )

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = TFExplorerNode()

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