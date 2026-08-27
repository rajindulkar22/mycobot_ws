#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointMonitorNode(Node):
    """Monitor myCobot joint positions and report safety conditions."""

    def __init__(self):
        super().__init__("joint_monitor")

        self.joint_limits = {
            "joint2_to_joint1": (-2.9321, 2.9321),
            "joint3_to_joint2": (-2.4434, 2.4434),
            "joint4_to_joint3": (-2.6179, 2.6179),
            "joint5_to_joint4": (-2.6179, 2.6179),
            "joint6_to_joint5": (-2.7052, 2.7925),
            "joint6output_to_joint6": (-3.14159, 3.14159),
        }

        self.short_names = {
            "joint2_to_joint1": "Joint 1",
            "joint3_to_joint2": "Joint 2",
            "joint4_to_joint3": "Joint 3",
            "joint5_to_joint4": "Joint 4",
            "joint6_to_joint5": "Joint 5",
            "joint6output_to_joint6": "Joint 6",
        }

        self.latest_positions = {}
        self.previous_status = {}

        self.message_count = 0
        self.previous_message_count = 0
        self.last_message_time = None

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.report_timer = self.create_timer(
            1.0,
            self.print_report,
        )

        self.get_logger().info(
            "Joint-state safety monitor started"
        )

    def joint_state_callback(self, message):
        """Store and validate every received joint-state message."""
        self.message_count += 1
        self.last_message_time = self.get_clock().now()

        if len(message.name) != len(message.position):
            self.get_logger().error(
                "JointState name and position lengths do not match"
            )
            return

        for joint_name, position in zip(
            message.name,
            message.position,
        ):
            if joint_name not in self.joint_limits:
                continue

            self.latest_positions[joint_name] = position
            status = self.calculate_status(
                joint_name,
                position,
            )

            previous = self.previous_status.get(joint_name)

            if status != previous:
                self.report_status_change(
                    joint_name,
                    position,
                    status,
                )

                self.previous_status[joint_name] = status

    def calculate_status(self, joint_name, position):
        """Classify one joint using its URDF limits."""
        lower, upper = self.joint_limits[joint_name]

        if position < lower or position > upper:
            return "INVALID"

        normalized = (
            (position - lower) /
            (upper - lower)
        )

        if normalized <= 0.10:
            return "NEAR_LOWER"

        if normalized >= 0.90:
            return "NEAR_UPPER"

        return "NORMAL"

    def calculate_percentage(self, joint_name, position):
        """Return the joint position as a percentage of its range."""
        lower, upper = self.joint_limits[joint_name]

        return (
            (position - lower) /
            (upper - lower)
        ) * 100.0

    def calculate_nearest_margin(self, joint_name, position):
        """Return angular distance to the nearest joint limit."""
        lower, upper = self.joint_limits[joint_name]

        return min(
            position - lower,
            upper - position,
        )

    def report_status_change(
        self,
        joint_name,
        position,
        status,
    ):
        """Log only when a joint changes safety state."""
        display_name = self.short_names[joint_name]
        degrees = math.degrees(position)

        if status == "INVALID":
            self.get_logger().error(
                f"{display_name} is outside its limits: "
                f"{degrees:.1f}°"
            )

        elif status == "NEAR_LOWER":
            self.get_logger().warning(
                f"{display_name} is near its lower limit: "
                f"{degrees:.1f}°"
            )

        elif status == "NEAR_UPPER":
            self.get_logger().warning(
                f"{display_name} is near its upper limit: "
                f"{degrees:.1f}°"
            )

        elif status == "NORMAL":
            self.get_logger().info(
                f"{display_name} returned to its normal range"
            )

    def print_report(self):
        """Print a one-second summary and check message health."""
        if self.last_message_time is None:
            self.get_logger().warning(
                "Waiting for /joint_states"
            )
            return

        current_time = self.get_clock().now()

        age_seconds = (
            current_time - self.last_message_time
        ).nanoseconds / 1_000_000_000

        messages_this_second = (
            self.message_count -
            self.previous_message_count
        )

        self.previous_message_count = self.message_count

        if age_seconds > 1.0:
            self.get_logger().error(
                f"Joint-state data is stale: "
                f"{age_seconds:.2f} seconds old"
            )
            return

        print("\nJoint safety report")
        print("-------------------")

        for joint_name in self.joint_limits:
            if joint_name not in self.latest_positions:
                print(
                    f"{self.short_names[joint_name]}: no data"
                )
                continue

            position = self.latest_positions[joint_name]
            percentage = self.calculate_percentage(
                joint_name,
                position,
            )
            margin = self.calculate_nearest_margin(
                joint_name,
                position,
            )
            status = self.calculate_status(
                joint_name,
                position,
            )

            print(
                f"{self.short_names[joint_name]}: "
                f"{math.degrees(position):7.1f}° | "
                f"{percentage:6.1f}% | "
                f"margin {math.degrees(margin):6.1f}° | "
                f"{status}"
            )

        print(
            f"Message rate: approximately "
            f"{messages_this_second} Hz"
        )

    def destroy_node(self):
        """Print a message before shutting down."""
        self.get_logger().info(
            "Joint-state safety monitor stopped"
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointMonitorNode()

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