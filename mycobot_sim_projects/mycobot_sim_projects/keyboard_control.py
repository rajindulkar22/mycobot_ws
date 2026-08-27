#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KeyboardControlNode(Node):
    """Control the six myCobot arm joints from the keyboard."""

    def __init__(self):
        super().__init__("keyboard_control")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
            "gripper_controller",
        ]

        self.lower_limits = [
            -2.9321,
            -2.4434,
            -2.6179,
            -2.6179,
            -2.7052,
            -3.14159,
            -0.5,
        ]

        self.upper_limits = [
            2.9321,
            2.4434,
            2.6179,
            2.6179,
            2.7925,
            3.14159,
            0.15,
        ]

        self.gripper_open_position = 0.0
        self.gripper_closed_position = -0.5

        self.positions = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            self.gripper_closed_position,
        ]

        self.selected_joint = 0
        self.step_size = math.radians(5.0)

        self.timer = self.create_timer(0.05, self.update)

        self.print_instructions()
        self.print_positions()

    def print_instructions(self):
        """Display available keyboard commands."""
        print(
            "\n"
            "myCobot keyboard controller\n"
            "---------------------------\n"
            "1-6 : select joint\n"
            "+/= : increase selected joint by 5 degrees\n"
            "-   : decrease selected joint by 5 degrees\n"
            "o   : open adaptive gripper\n"
            "c   : close adaptive gripper\n"
            "h   : return all joints home\n"
            "p   : print joint positions\n"
            "q   : quit\n"
        )

    def read_key(self):
        """Return a pressed key without blocking the ROS timer."""
        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.0,
        )

        if readable:
            return sys.stdin.read(1)

        return None

    def update(self):
        """Read keyboard input and publish the current configuration."""
        key = self.read_key()

        if key is not None:
            self.process_key(key)

        if rclpy.ok():
            self.publish_joint_state()

    def process_key(self, key):
        """Apply a keyboard command."""
        if key in "123456":
            self.selected_joint = int(key) - 1

            self.get_logger().info(
                f"Selected joint {self.selected_joint + 1}: "
                f"{self.joint_names[self.selected_joint]}"
            )

        elif key in ("+", "="):
            self.change_selected_joint(self.step_size)

        elif key == "-":
            self.change_selected_joint(-self.step_size)

        elif key.lower() == "o":
            self.positions[6] = self.gripper_open_position
            self.get_logger().info("Gripper opened")

        elif key.lower() == "c":
            self.positions[6] = self.gripper_closed_position
            self.get_logger().info("Gripper closed")

        elif key.lower() == "h":
            self.positions = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                self.gripper_closed_position,
            ]

            self.get_logger().info(
                "Returned arm home and closed gripper"
            )
            self.print_positions()

        elif key.lower() == "p":
            self.print_positions()

        elif key.lower() == "q":
            self.get_logger().info("Stopping keyboard controller")
            rclpy.shutdown()

    def change_selected_joint(self, change):
        """Change one joint while enforcing its URDF limits."""
        index = self.selected_joint
        requested_position = self.positions[index] + change

        clamped_position = max(
            self.lower_limits[index],
            min(requested_position, self.upper_limits[index]),
        )

        self.positions[index] = clamped_position

        if clamped_position != requested_position:
            self.get_logger().warning(
                f"Joint {index + 1} reached its limit"
            )

        self.get_logger().info(
            f"Joint {index + 1}: "
            f"{math.degrees(clamped_position):.1f} degrees"
        )

    def publish_joint_state(self):
        """Publish all arm and gripper joint positions."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = self.positions

        self.publisher.publish(message)

    def print_positions(self):
        """Print arm and gripper positions in degrees."""
        print("\nCurrent arm positions:")

        for index, position in enumerate(self.positions[:6]):
            print(
                f"  Joint {index + 1}: "
                f"{math.degrees(position):7.1f}°"
            )

        print(
            "  Gripper: "
            f"{math.degrees(self.positions[6]):7.1f}°"
        )
        print()


def main(args=None):
    if not sys.stdin.isatty():
        print(
            "Error: keyboard control requires an interactive terminal.",
            file=sys.stderr,
        )
        return

    original_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init(args=args)
        node = KeyboardControlNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )


if __name__ == "__main__":
    main()