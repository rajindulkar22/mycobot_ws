#!/usr/bin/env python3
"""Command the simulated myCobot adaptive gripper using a ROS 2 action."""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


# Full mechanical range of the gripper_controller joint, taken from the
# URDF. Wider than the -0.5/0.0 closed/open presets keyboard_control.py
# actually commands — this file allows any position in the full range via
# the 'custom' command.
GRIPPER_MIN = -0.74
GRIPPER_MAX = 0.15

# Convenience presets in radians. 'half' (-0.25) isn't the midpoint of
# GRIPPER_MIN/GRIPPER_MAX; it's roughly halfway between the practical
# open (0.0) and closed (-0.5) positions.
NAMED_COMMANDS = {
    "open": 0.0,
    "wide": 0.12,
    "half": -0.25,
    "close": -0.5,
}


class GripperCommander(Node):
    """Send position goals to the adaptive-gripper action controller."""

    def __init__(self):
        super().__init__("gripper_commander")

        # GripperCommand is the standard ros2_control action for a single
        # position+effort target, as opposed to FollowJointTrajectory's
        # multi-joint, multi-waypoint trajectory.
        self._client = ActionClient(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
        )

    def execute(
        self,
        command_name: str,
        position: float,
        max_effort: float,
    ) -> bool:
        """Validate and execute one gripper command."""

        self._validate_command(position, max_effort)

        self.get_logger().info(
            "Waiting for "
            "/gripper_action_controller/gripper_cmd ..."
        )

        # Fail fast with a clear message if gripper_action_controller was
        # never spawned/activated, rather than hanging indefinitely.
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Gripper action server is unavailable. "
                "Check that gripper_action_controller is active."
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        self.get_logger().info(
            f"Sending '{command_name}' command: "
            f"position={position:.3f} rad, "
            f"max_effort={max_effort:.2f}"
        )

        # Step 1 of the action handshake: send the goal and register a
        # feedback callback (fired repeatedly while the goal executes),
        # then block until the server has decided whether to accept it.
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "The gripper controller returned no goal handle."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "The gripper controller rejected the goal."
            )
            return False

        self.get_logger().info(
            "Goal accepted; waiting for completion ..."
        )

        # Step 2: block again, this time until the gripper has actually
        # finished moving (reached the target, stalled, or was cancelled).
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "The gripper controller returned no result."
            )
            return False

        result = wrapped_result.result
        status = wrapped_result.status

        self.get_logger().info(
            "Gripper result: "
            f"position={result.position:.3f} rad, "
            f"effort={result.effort:.3f}, "
            f"stalled={result.stalled}, "
            f"reached_goal={result.reached_goal}"
        )

        # `status` is the action-level outcome (accepted/executing/
        # succeeded/aborted/canceled). It can be non-SUCCEEDED even though
        # a `result` message came back, so it's checked before trusting
        # reached_goal/stalled below.
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Gripper action finished with status {status}."
            )
            return False

        # Two different "good" outcomes, per the stall model documented in
        # THEORY.md: reached_goal means the fingers got to `position`
        # cleanly; stalled means motion stopped early (e.g. against an
        # object) and allow_stalling let that count as success anyway.
        if result.reached_goal:
            self.get_logger().info(
                f"Command '{command_name}' completed: "
                "target position reached."
            )

        elif result.stalled:
            self.get_logger().info(
                f"Command '{command_name}' completed: "
                "gripper stalled, possibly due to object contact."
            )

        else:
            # STATUS_SUCCEEDED but neither flag set shouldn't normally
            # happen; logged as a warning rather than treated as failure
            # since the action itself still reported success.
            self.get_logger().warning(
                f"Command '{command_name}' completed, "
                "but neither reached_goal nor stalled was reported."
            )

        return True

    def _feedback_callback(self, feedback_message):
        """Print live gripper-controller feedback."""

        # Called repeatedly by rclpy while the goal is executing (not just
        # once at the end), so the operator can watch the gripper close in
        # real time rather than waiting silently for the final result.
        feedback = feedback_message.feedback

        self.get_logger().info(
            "Feedback: "
            f"position={feedback.position:.3f} rad, "
            f"effort={feedback.effort:.3f}, "
            f"stalled={feedback.stalled}, "
            f"reached_goal={feedback.reached_goal}"
        )

    @staticmethod
    def _validate_command(
        position: float,
        max_effort: float,
    ):
        """Reject invalid commands before sending them."""

        # Catches nan/inf, which would otherwise pass a naive
        # GRIPPER_MIN <= position <= GRIPPER_MAX comparison in unexpected
        # ways.
        if not math.isfinite(position):
            raise ValueError(
                "Gripper position must be finite."
            )

        if position < GRIPPER_MIN or position > GRIPPER_MAX:
            raise ValueError(
                f"Position {position:.3f} rad is outside "
                f"[{GRIPPER_MIN:.3f}, {GRIPPER_MAX:.3f}]."
            )

        if not math.isfinite(max_effort):
            raise ValueError(
                "Maximum effort must be finite."
            )

        if max_effort < 0.0:
            raise ValueError(
                "Maximum effort cannot be negative."
            )


def parse_arguments():
    """Read the gripper command and optional parameters."""

    parser = argparse.ArgumentParser(
        description=(
            "Open, close or partially close "
            "the simulated adaptive gripper."
        )
    )

    parser.add_argument(
        "command",
        choices=[
            "open",
            "wide",
            "half",
            "close",
            "custom",
        ],
        help="Named gripper command.",
    )

    parser.add_argument(
        "--position",
        type=float,
        default=None,
        help=(
            "Custom position in radians. "
            "Required when command is 'custom'."
        ),
    )

    parser.add_argument(
        "--max-effort",
        type=float,
        default=5.0,
        help="Maximum gripping effort (default: 5.0).",
    )

    # `ros2 run` appends ROS-specific args (e.g. __node:=..., __ns:=...) to
    # sys.argv; remove_ros_args strips those out so argparse only sees the
    # arguments this script actually defines. [1:] drops the program name.
    application_arguments = remove_ros_args(
        args=sys.argv
    )[1:]

    parsed = parser.parse_args(
        application_arguments
    )

    # --position is only meaningful (and required) for 'custom'; enforcing
    # that pairing here means NAMED_COMMANDS stays the single source of
    # truth for the three preset positions instead of --position silently
    # overriding them.
    if parsed.command == "custom":
        if parsed.position is None:
            parser.error(
                "'custom' requires --position."
            )

    elif parsed.position is not None:
        parser.error(
            "--position can only be used "
            "with the 'custom' command."
        )

    return parsed


def main(args=None):
    """Program entry point."""

    parsed = parse_arguments()

    if parsed.command == "custom":
        position = parsed.position
        command_name = "custom"

    else:
        # Named commands look up their position from NAMED_COMMANDS;
        # 'custom' is the only one that takes it from the CLI directly.
        position = NAMED_COMMANDS[parsed.command]
        command_name = parsed.command

    rclpy.init(args=args)

    node = GripperCommander()
    exit_code = 1

    try:
        success = node.execute(
            command_name=command_name,
            position=position,
            max_effort=parsed.max_effort,
        )

        exit_code = 0 if success else 1

    except ValueError as error:
        # Raised by _validate_command for an out-of-range/non-finite
        # position or a negative max_effort.
        node.get_logger().error(
            str(error)
        )

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Gripper command cancelled by user."
        )

    finally:
        # Always clean up, even if execute() raised or was interrupted, so
        # the process doesn't leave a dangling node/context behind.
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
