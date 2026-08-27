#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PoseSequenceNode(Node):
    """Publish smoothly interpolated myCobot joint configurations."""

    def __init__(self):
        super().__init__("pose_sequence")

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
        ]

        self.poses = [
            (
                "Home",
                self.degrees_to_radians([0, 0, 0, 0, 0, 0]),
            ),
            (
                "Observe",
                self.degrees_to_radians([0, -25, -25, 0, 25, 0]),
            ),
            (
                "Pick-ready",
                self.degrees_to_radians([20, -35, 30, 0, -20, 0]),
            ),
        ]

        # Motion tuning: seconds to glide between two poses, seconds to pause once
        # arrived, and how many times per second update_motion() recomputes/publishes.
        self.transition_duration = 3.0
        self.hold_duration = 1.5
        self.publish_frequency = 20.0

        # Start parked at the "Home" pose (poses[0]) — this is what actually gets published.
        self.current_positions = list(self.poses[0][1])
        # Snapshot of where the *current* transition began, used as the lerp start point.
        self.start_positions = list(self.current_positions)

        # First move is from Home (index 0) to poses[1] ("Observe").
        self.target_index = 1
        self.target_name = self.poses[self.target_index][0]
        self.target_positions = list(self.poses[self.target_index][1])

        # transition_start_time anchors the elapsed-time calc in update_motion();
        # hold_start_time/is_holding track whether we're mid-move or paused at a pose.
        self.transition_start_time = self.get_clock().now()
        self.hold_start_time = None
        self.is_holding = False

        # e.g. 20 Hz -> fire update_motion() every 0.05s for a smooth glide.
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(
            timer_period,
            self.update_motion,
        )

        self.get_logger().info("Smooth pose sequence started")
        self.get_logger().info(
            f"Moving from Home to {self.target_name}"
        )

    @staticmethod
    def degrees_to_radians(angles_degrees):
        """Convert a list of angles from degrees to radians."""
        return [math.radians(angle) for angle in angles_degrees]

    def publish_joint_state(self):
        """Publish the current six joint positions."""
        # Always publishes self.current_positions — whatever update_motion() last
        # computed, whether that's a mid-glide interpolated value or a held pose.
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = self.current_positions

        self.publisher.publish(message)

    def update_motion(self):
        """Calculate and publish the next interpolated configuration."""
        # Timer callback, runs at publish_frequency Hz. Acts as a small state machine
        # with two states: holding (paused at a pose) and transitioning (gliding).
        current_time = self.get_clock().now()

        if self.is_holding:
            # Just keep re-publishing the same held pose until hold_duration elapses.
            self.publish_joint_state()

            held_seconds = (
                current_time - self.hold_start_time
            ).nanoseconds / 1_000_000_000

            if held_seconds >= self.hold_duration:
                self.start_next_transition(current_time)

            return

        # In transition: alpha is progress through the glide, 0.0 (just started)
        # to 1.0 (arrived), clamped so overshoot past transition_duration doesn't
        # extrapolate beyond the target.
        elapsed_seconds = (
            current_time - self.transition_start_time
        ).nanoseconds / 1_000_000_000

        alpha = min(
            elapsed_seconds / self.transition_duration,
            1.0,
        )

        # Linear interpolation (lerp) per joint: start + alpha * (target - start).
        # zip pairs up each joint's start/target angle so all six move in lockstep.
        self.current_positions = [
            start + alpha * (target - start)
            for start, target in zip(
                self.start_positions,
                self.target_positions,
            )
        ]

        self.publish_joint_state()

        if alpha >= 1.0:
            # Snap exactly onto the target (avoids float drift from the lerp math),
            # then switch into the holding state.
            self.current_positions = list(self.target_positions)
            self.is_holding = True
            self.hold_start_time = current_time

            self.get_logger().info(
                f"Reached {self.target_name}"
            )

    def start_next_transition(self, current_time):
        """Select the next pose and start moving towards it."""
        # New glide starts from wherever we currently are...
        self.start_positions = list(self.current_positions)

        # ...and ends at the next pose in the list, wrapping back to index 0
        # (Home) once the sequence finishes Pick-ready.
        self.target_index = (
            self.target_index + 1
        ) % len(self.poses)

        self.target_name = self.poses[self.target_index][0]
        self.target_positions = list(
            self.poses[self.target_index][1]
        )

        # Reset the transition clock and drop out of the holding state.
        self.transition_start_time = current_time
        self.is_holding = False
        self.hold_start_time = None

        self.get_logger().info(
            f"Moving to {self.target_name}"
        )


def main(args=None):
    # Standard ROS 2 entry point: init, construct node, spin until interrupted, then clean up.
    rclpy.init(args=args)

    node = PoseSequenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Only runs main() when executed directly, not when imported by a ROS 2 entry-point wrapper.
    main()