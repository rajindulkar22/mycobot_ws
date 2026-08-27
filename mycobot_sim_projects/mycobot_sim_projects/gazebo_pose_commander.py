#!/usr/bin/env python3
"""Send safe named poses to the myCobot Gazebo trajectory controller."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Six arm joints, in the order the arm_controller (JointTrajectoryController)
# expects them. Order matters: index i here must line up with index i in
# every POSES entry and in JOINT_LIMITS below.
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Limits copied from the myCobot 280 JN adaptive-gripper URDF, in radians.
# Used client-side in _validate_pose() so an out-of-range pose is rejected
# before it's ever sent to the controller, instead of failing inside Gazebo.
JOINT_LIMITS = [
    (-2.9321, 2.9321),
    (-2.4434, 2.4434),
    (-2.6179, 2.6179),
    (-2.6179, 2.6179),
    (-2.7052, 2.7925),
    (-3.14159, 3.14159),
]

# Named joint-space configurations, in radians. Each list is a full 6-DOF
# target, one value per JOINT_NAMES entry, chosen by hand rather than by IK.
#
# Top-down pick for 25 mm pick_cube at (0.20, 0.00, 0.4125) m in Gazebo:
#   pick_cube: open → approach → hover → descend → close → lift
# FK targets (joint1 frame): cube top z≈0.020 m; descend TCP z≈0.054 m (~34 mm
# above top) so open fingers clear the sides, then close grips the block.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = -0.55
GRIPPER_MAX_EFFORT = 8.0
GRIPPER_HOLD_SEC = 0.8
GRIPPER_OPEN_TIMEOUT_SEC = 10.0
GRIPPER_CLOSE_TIMEOUT_SEC = 3.0
DESCEND_DURATION_SCALE = 0.7
DESCEND_MIN_DURATION_SEC = 3.5
LIFT_DURATION_SCALE = 0.8
LIFT_MIN_DURATION_SEC = 4.0
POSES = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, -0.436332, -0.959931, 0.0, 0.698132, 0.0],
    "observe": [0.0, -0.70, -0.70, 0.0, 0.60, 0.0],
    "left_ready": [0.523599, -0.610865, -0.698132, 0.0, 0.610865, 0.0],

    # --- Side / experimental picks (older, keep for reference) ---
    "pick_approach": [
        -0.132470,
        -0.315206,
        -1.442864,
        0.249233,
        0.713840,
        -0.172962,
    ],
    "pick_pregrasp": [
        -0.132470,
        -0.450120,
        -1.696809,
        0.638267,
        0.713840,
        -0.172962,
    ],
    "pick_test": [
        -0.132470,
        -0.703019,
        -1.800656,
        0.995012,
        0.713840,
        -0.172962,
    ],

    # --- Top-down pick over 25 mm pick_cube at (0.20, 0.00, 0.4125) m ---
    # joint1≈0.33 centres TCP in XY for the top-down wrist (j5≈0, j6≈−71°).
    "grasp_approach": [
        0.330216,
        -0.212581,
        -1.602910,
        0.241554,
        -0.007679,
        -1.240580,
    ],
    # Mid descend — FK tcp z≈0.105 m (well above the cube).
    "grasp_hover": [
        0.330216,
        -0.407500,
        -1.711000,
        0.520000,
        -0.007679,
        -1.240580,
    ],
    # Pre-close height — FK tcp z≈0.054 m (~34 mm above 25 mm cube top).
    "grasp_descend": [
        0.330216,
        -0.700000,
        -1.875000,
        0.938000,
        -0.007679,
        -1.240580,
    ],
    # Lift in two vertical stages (reverse of descend): hover clears table,
    # then retract to carry height. Do not jump descend → approach in one move.
    "grasp_lift": [
        0.330216,
        -0.407500,
        -1.711000,
        0.520000,
        -0.007679,
        -1.240580,
    ],
    "grasp_retract": [
        0.330216,
        -0.212581,
        -1.602910,
        0.241554,
        -0.007679,
        -1.240580,
    ],

    "grasp": [
        0.330216,
        -0.700000,
        -1.875000,
        0.938000,
        -0.007679,
        -1.240580,
    ],
}

# Older names map to the safe poses above.
POSES["top_pick_approach_calibrated"] = list(POSES["grasp_approach"])
POSES["top_pick_pregrasp_calibrated"] = list(POSES["grasp_descend"])

GRASP_SEQUENCE = (
    "grasp_approach",
    "grasp_hover",
    "grasp_descend",
)

LIFT_SEQUENCE = (
    "grasp_lift",
    "grasp_retract",
)


class GazeboPoseCommander(Node):
    """Action client for the arm joint-trajectory controller."""

    def __init__(self) -> None:
        super().__init__("gazebo_pose_commander")

        # FollowJointTrajectory is the standard ros2_control action for
        # executing a trajectory over time (as opposed to the earlier
        # projects, which publish raw /joint_states and skip the controller
        # and physics simulation entirely).
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
        )

    def execute(self, pose_name: str, duration: float) -> bool:
        """Validate and execute one named pose."""
        positions = POSES[pose_name]
        self._validate_pose(pose_name, positions)

        self.get_logger().info(
            "Waiting for /arm_controller/follow_joint_trajectory ..."
        )
        # Fail fast with a clear message if arm_controller was never
        # spawned/activated, rather than hanging indefinitely.
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Trajectory action server is unavailable. "
                "Check that arm_controller is active."
            )
            return False

        # A trajectory names which joints it drives, plus one or more
        # waypoints. This node only ever sends a single waypoint: go
        # straight from wherever the arm currently is to `positions`,
        # arriving at `duration` seconds from now.
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        # Zero target velocity means "come to a stop exactly at this point"
        # rather than "pass through it while still moving".
        point.velocities = [0.0] * len(JOINT_NAMES)

        # time_from_start is a ROS Duration (separate integer seconds and
        # nanoseconds fields), so the float `duration` has to be split.
        whole_seconds = int(duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (duration - whole_seconds) * 1_000_000_000
        )
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        degrees = [round(math.degrees(value), 1) for value in positions]
        self.get_logger().info(
            f"Sending pose '{pose_name}' over {duration:.1f} s: {degrees} deg"
        )

        # Step 1 of the action handshake: send the goal and block until the
        # server has decided whether to accept it (not until it's finished
        # executing). send_goal_async() itself is non-blocking; it's
        # spin_until_future_complete() that actually waits.
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        # A goal can be rejected for reasons like: wrong/missing joint
        # names, the controller being inactive, a malformed trajectory, or
        # an incompatible goal already executing.
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("The controller rejected the trajectory.")
            return False

        # Step 2: block again, this time until the trajectory has actually
        # finished executing in Gazebo (succeeded, failed, was cancelled, or
        # violated a tolerance) rather than just been accepted.
        self.get_logger().info("Goal accepted; waiting for completion ...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("No result was returned by the controller.")
            return False

        result = wrapped_result.result
        # SUCCESSFUL is one specific enum value; anything else (e.g. a path
        # tolerance violation, or the goal being pre-empted) is a failure.
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "Trajectory failed with error code "
                f"{result.error_code}: {result.error_string}"
            )
            return False

        self.get_logger().info(f"Pose '{pose_name}' completed successfully.")
        return True

    def _send_gripper_goal(
        self,
        position: float,
        label: str,
        *,
        timeout_sec: float,
        proceed_on_timeout: bool = False,
    ) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Gripper action server is unavailable. "
                "Check that gripper_action_controller is active."
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        self.get_logger().info(
            f"Gripper '{label}': position={position:.2f} rad ..."
        )
        send_future = self._gripper_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec):
            self.get_logger().error(f"Gripper '{label}' goal send timed out.")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Gripper '{label}' goal was rejected.")
            return False

        self.get_logger().info(
            f"Gripper '{label}' accepted; waiting up to {timeout_sec:.1f} s ..."
        )

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec):
            if proceed_on_timeout:
                self.get_logger().warning(
                    f"Gripper '{label}' timed out after {timeout_sec:.1f} s "
                    "(common when fingers stall on the cube); continuing pick."
                )
                return True
            self.get_logger().error(
                f"Gripper '{label}' did not finish within {timeout_sec:.1f} s."
            )
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(f"Gripper '{label}' returned no result.")
            return False

        result = wrapped_result.result
        status = wrapped_result.status
        self.get_logger().info(
            f"Gripper '{label}' result: position={result.position:.3f} rad, "
            f"effort={result.effort:.3f}, stalled={result.stalled}, "
            f"reached_goal={result.reached_goal}, status={status}"
        )

        if status != GoalStatus.STATUS_SUCCEEDED:
            if proceed_on_timeout and (result.stalled or result.reached_goal):
                return True
            self.get_logger().error(
                f"Gripper '{label}' finished with action status {status}."
            )
            return False

        if not (result.reached_goal or result.stalled):
            if proceed_on_timeout:
                self.get_logger().warning(
                    f"Gripper '{label}' finished without reached_goal/stall; "
                    "continuing pick anyway."
                )
                return True
            self.get_logger().warning(
                f"Gripper '{label}' finished without reached_goal or stall."
            )
        return True

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return future.done()

    def open_gripper(self) -> bool:
        """Open the adaptive gripper to the normal preset before a pick."""
        return self._send_gripper_goal(
            GRIPPER_OPEN,
            "open",
            timeout_sec=GRIPPER_OPEN_TIMEOUT_SEC,
        )

    def close_gripper(self) -> bool:
        """Close the adaptive gripper on the object at grasp height."""
        return self._send_gripper_goal(
            GRIPPER_CLOSED,
            "close",
            timeout_sec=GRIPPER_CLOSE_TIMEOUT_SEC,
            proceed_on_timeout=True,
        )

    def _step_duration(self, pose_name: str, duration: float) -> float:
        if pose_name == "grasp_descend":
            return max(duration * DESCEND_DURATION_SCALE, DESCEND_MIN_DURATION_SEC)
        if pose_name == "grasp_hover":
            return max(duration * DESCEND_DURATION_SCALE, 2.5)
        if pose_name in LIFT_SEQUENCE:
            return max(duration * LIFT_DURATION_SCALE, LIFT_MIN_DURATION_SEC)
        return duration

    def execute_pick_cube(self, duration: float) -> bool:
        """Full top-down pick: open → approach → descend → close → lift."""
        if not self.open_gripper():
            return False

        self.get_logger().info(
            "Running top-down pick: "
            + " → ".join(GRASP_SEQUENCE)
            + " → close → "
            + " → ".join(LIFT_SEQUENCE)
        )
        for pose_name in GRASP_SEQUENCE:
            if not self.execute(pose_name, self._step_duration(pose_name, duration)):
                self.get_logger().error(
                    f"Pick stopped at arm pose '{pose_name}'."
                )
                return False

        if not self.close_gripper():
            return False

        if GRIPPER_HOLD_SEC > 0.0:
            self.get_logger().info(
                f"Holding grasp for {GRIPPER_HOLD_SEC:.1f} s ..."
            )
            time.sleep(GRIPPER_HOLD_SEC)

        for pose_name in LIFT_SEQUENCE:
            if not self.execute(pose_name, self._step_duration(pose_name, duration)):
                self.get_logger().error(f"Pick stopped at lift pose '{pose_name}'.")
                return False

        self.get_logger().info(
            "pick_cube completed (approach → descend → close → lift → retract)."
        )
        return True

    def execute_grasp_approach(self, duration: float) -> bool:
        """Arm-only top-down approach: open → approach → descend."""
        if not self.open_gripper():
            return False

        self.get_logger().info(
            "Running arm-only top-down approach: "
            + " → ".join(GRASP_SEQUENCE)
        )
        for pose_name in GRASP_SEQUENCE:
            if not self.execute(pose_name, self._step_duration(pose_name, duration)):
                self.get_logger().error(
                    f"Grasp sequence stopped at '{pose_name}'."
                )
                return False
        self.get_logger().info(
            "Arm is at 'grasp_descend'. Close and lift manually, e.g.:\n"
            "  ros2 run mycobot_sim_projects gripper_commander -- close\n"
            "  ros2 run mycobot_sim_projects gazebo_pose_commander "
            "grasp_lift\n"
            "  ros2 run mycobot_sim_projects gazebo_pose_commander "
            "grasp_retract"
        )
        return True

    @staticmethod
    def _validate_pose(pose_name: str, positions: list[float]) -> None:
        # Guards against a POSES entry that's missing a joint or has an
        # extra one, which would otherwise silently misalign with
        # JOINT_NAMES when the trajectory is built.
        if len(positions) != len(JOINT_NAMES):
            raise ValueError(
                f"Pose '{pose_name}' contains {len(positions)} values; "
                f"expected {len(JOINT_NAMES)}."
            )

        for joint_name, value, limits in zip(
            JOINT_NAMES, positions, JOINT_LIMITS
        ):
            lower, upper = limits
            # Catches nan/inf, which would otherwise pass a naive
            # lower <= value <= upper comparison in unexpected ways.
            if not math.isfinite(value):
                raise ValueError(f"{joint_name} has a non-finite value.")
            if value < lower or value > upper:
                raise ValueError(
                    f"Pose '{pose_name}' violates {joint_name}: "
                    f"{value:.4f} rad is outside [{lower:.4f}, {upper:.4f}]."
                )


def parse_arguments() -> argparse.Namespace:
    """Parse application arguments while ignoring ROS-specific arguments."""
    pose_choices = sorted(POSES) + ["pick_cube"]
    parser = argparse.ArgumentParser(
        description=(
            "Move the Gazebo myCobot arm to a named pose. "
            "Use 'pick_cube' for the full pick (through close and lift)."
        )
    )
    parser.add_argument(
        "pose",
        choices=pose_choices,
        help="Named target pose, or 'pick_cube' for the safe approach chain.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Trajectory duration in seconds (default: 5.0).",
    )
    # `ros2 run` appends ROS-specific args (e.g. __node:=..., __ns:=...) to
    # sys.argv; remove_ros_args strips those out so argparse only sees the
    # arguments this script actually defines. [1:] drops the program name.
    arguments = remove_ros_args(args=sys.argv)[1:]
    parsed = parser.parse_args(arguments)
    if parsed.duration <= 0.0:
        parser.error("--duration must be greater than zero")
    return parsed


def main(args=None) -> None:
    parsed = parse_arguments()
    rclpy.init(args=args)
    node = GazeboPoseCommander()
    exit_code = 1

    try:
        if parsed.pose == "pick_cube":
            ok = node.execute_pick_cube(parsed.duration)
        else:
            ok = node.execute(parsed.pose, parsed.duration)
        exit_code = 0 if ok else 1
    except (KeyError, ValueError) as error:
        # KeyError: pose name not in POSES (shouldn't happen given argparse's
        # `choices=...`, but kept as a defensive backstop).
        # ValueError: raised by _validate_pose for a malformed/out-of-limits
        # pose.
        node.get_logger().error(str(error))
    except KeyboardInterrupt:
        node.get_logger().warning("Pose command cancelled by user.")
    finally:
        # Always clean up, even if execute() raised or was interrupted, so
        # the process doesn't leave a dangling node/context behind.
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
