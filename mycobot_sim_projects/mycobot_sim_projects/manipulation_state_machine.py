#!/usr/bin/env python3
"""Coordinate the myCobot arm and adaptive gripper using a state machine."""

import argparse
import sys
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import (
    FollowJointTrajectory,
    GripperCommand,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


ARM_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


# A local copy of named poses, not imported from gazebo_pose_commander.py,
# so this node stays runnable on its own. For table-safe cube picking, use
# gazebo_pose_commander.py's grasp_* poses / pick_cube sequence instead.
ARM_POSES = {
    "home": [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],

    "ready": [
        0.0,
        -0.436332,
        -0.959931,
        0.0,
        0.698132,
        0.0,
    ],

    "approach": [
        0.0,
        -0.70,
        -0.70,
        0.0,
        0.60,
        0.0,
    ],

    "lift": [
        0.0,
        -0.45,
        -0.80,
        0.0,
        0.50,
        0.0,
    ],
}


GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = -0.5
GRIPPER_MAX_EFFORT = 5.0


class TaskState(Enum):
    """Possible manipulation-task states."""

    # Happy-path states, executed roughly in this order by run().
    HOME = auto()
    OPEN_GRIPPER = auto()
    READY = auto()
    APPROACH = auto()
    CLOSE_GRIPPER = auto()
    LIFT = auto()
    RELEASE = auto()
    RETURN_HOME = auto()
    COMPLETE = auto()
    # Reachable from any happy-path state via fail(), not from each other
    # in sequence: RECOVERY always leads straight to FAILED.
    RECOVERY = auto()
    FAILED = auto()


# Happy-path order used by the GUI to show progress.
TASK_FLOW = (
    TaskState.HOME,
    TaskState.OPEN_GRIPPER,
    TaskState.READY,
    TaskState.APPROACH,
    TaskState.CLOSE_GRIPPER,
    TaskState.LIFT,
    TaskState.RELEASE,
    TaskState.RETURN_HOME,
    TaskState.COMPLETE,
)


class ManipulationStateMachine(Node):
    """Coordinate arm and gripper action clients."""

    def __init__(
        self,
        arm_duration: float,
        pause_duration: float,
    ):
        super().__init__("manipulation_state_machine")

        self.arm_duration = arm_duration
        self.pause_duration = pause_duration

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
        )

        self.state = TaskState.HOME
        # Set by fail() before it overwrites self.state via transition();
        # kept around so the final FAILED log can report which step in the
        # sequence actually broke, not just that the run failed.
        self.failed_state = None
        self.stop_requested = False
        self.is_running = False

    def wait_for_controllers(self) -> bool:
        """Wait until both action servers are available."""

        self.get_logger().info(
            "Waiting for arm trajectory controller..."
        )

        arm_available = self.arm_client.wait_for_server(
            timeout_sec=10.0
        )

        self.get_logger().info(
            "Waiting for gripper controller..."
        )

        # Sequential, not parallel: worst case this waits up to 10s for the
        # arm server before even starting the 10s wait for the gripper
        # server, so a cold controller_manager can take ~20s here.
        gripper_available = self.gripper_client.wait_for_server(
            timeout_sec=10.0
        )

        if not arm_available:
            self.get_logger().error(
                "Arm action server is unavailable."
            )

        if not gripper_available:
            self.get_logger().error(
                "Gripper action server is unavailable."
            )

        return arm_available and gripper_available

    def reset(self) -> None:
        """Return the task to its initial state."""

        self.state = TaskState.HOME
        self.failed_state = None
        self.stop_requested = False

    def request_stop(self) -> None:
        """Ask the running sequence to exit after the current step."""

        self.stop_requested = True

    @property
    def status_label(self) -> str:
        """Short status string for GUIs and logging."""

        if self.state == TaskState.FAILED and self.failed_state:
            return f"FAILED (during {self.failed_state})"
        return self.state.name

    def run(self) -> bool:
        """Execute the complete manipulation sequence."""

        self.is_running = True
        self.stop_requested = False

        try:
            if not self.wait_for_controllers():
                return False

            self.get_logger().info(
                "Starting manipulation state machine."
            )

            # The state machine itself: one big dispatch on self.state per
            # iteration. Each happy-path branch performs one arm/gripper move
            # and either transitions forward on success or calls fail() on
            # failure. COMPLETE and FAILED are the only branches that return,
            # ending the loop.
            while rclpy.ok():
                if self.stop_requested:
                    self.get_logger().warning(
                        "Manipulation sequence stopped by user."
                    )
                    self.transition(TaskState.FAILED)
                    self.failed_state = "STOPPED"
                    return False

                self.print_state()

                if self.state == TaskState.HOME:
                    if self.move_arm("home"):
                        self.transition(TaskState.OPEN_GRIPPER)
                    else:
                        self.fail()

                elif self.state == TaskState.OPEN_GRIPPER:
                    if self.move_gripper(
                        name="open",
                        position=GRIPPER_OPEN,
                    ):
                        self.transition(TaskState.READY)
                    else:
                        self.fail()

                elif self.state == TaskState.READY:
                    if self.move_arm("ready"):
                        self.transition(TaskState.APPROACH)
                    else:
                        self.fail()

                elif self.state == TaskState.APPROACH:
                    if self.move_arm("approach"):
                        self.transition(TaskState.CLOSE_GRIPPER)
                    else:
                        self.fail()

                elif self.state == TaskState.CLOSE_GRIPPER:
                    if self.move_gripper(
                        name="close",
                        position=GRIPPER_CLOSED,
                    ):
                        self.transition(TaskState.LIFT)
                    else:
                        self.fail()

                elif self.state == TaskState.LIFT:
                    if self.move_arm("lift"):
                        self.transition(TaskState.RELEASE)
                    else:
                        self.fail()

                elif self.state == TaskState.RELEASE:
                    if self.move_gripper(
                        name="release",
                        position=GRIPPER_OPEN,
                    ):
                        self.transition(TaskState.RETURN_HOME)
                    else:
                        self.fail()

                elif self.state == TaskState.RETURN_HOME:
                    if self.move_arm("home"):
                        self.transition(TaskState.COMPLETE)
                    else:
                        self.fail()

                elif self.state == TaskState.COMPLETE:
                    self.get_logger().info(
                        "Manipulation sequence completed successfully."
                    )
                    return True

                elif self.state == TaskState.RECOVERY:
                    # Always followed by FAILED: recovery only tries to leave
                    # the robot in a safe physical state, it doesn't undo the
                    # fact that the task itself didn't complete.
                    self.execute_recovery()
                    self.transition(TaskState.FAILED)

                elif self.state == TaskState.FAILED:
                    self.get_logger().error(
                        "Manipulation sequence failed. "
                        f"Original failed state: {self.failed_state}"
                    )
                    return False

            return False
        finally:
            self.is_running = False

    def move_arm(self, pose_name: str) -> bool:
        """Move the six arm joints to one named pose."""

        positions = ARM_POSES[pose_name]

        # Same single-waypoint trajectory shape as gazebo_pose_commander.py:
        # one JointTrajectoryPoint, zero final velocity, time_from_start
        # split into whole seconds and nanoseconds.
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(
            ARM_JOINT_NAMES
        )

        whole_seconds = int(self.arm_duration)
        nanoseconds = int(
            (
                self.arm_duration
                - whole_seconds
            )
            * 1_000_000_000
        )

        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"Moving arm to '{pose_name}' "
            f"over {self.arm_duration:.1f} seconds."
        )

        # Step 1: send the goal, block until the server accepts/rejects it.
        send_future = self.arm_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "Arm controller returned no goal handle."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Arm pose '{pose_name}' was rejected."
            )
            return False

        # Step 2: block until the trajectory has actually finished
        # executing (not just been accepted).
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "Arm controller returned no result."
            )
            return False

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                f"Arm pose '{pose_name}' failed with "
                f"action status {wrapped_result.status}."
            )
            return False

        result = wrapped_result.result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error(
                f"Arm pose '{pose_name}' failed: "
                f"error_code={result.error_code}, "
                f"message='{result.error_string}'."
            )
            return False

        self.get_logger().info(
            f"Arm reached '{pose_name}'."
        )

        # Brief pause between task steps (configurable via --pause), purely
        # for readability of the demo — not required for correctness.
        self.pause()
        return True

    def move_gripper(
        self,
        name: str,
        position: float,
    ) -> bool:
        """Open or close the adaptive gripper."""

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = (
            GRIPPER_MAX_EFFORT
        )

        self.get_logger().info(
            f"Sending gripper command '{name}': "
            f"{position:.3f} rad."
        )

        send_future = (
            self.gripper_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "Gripper controller returned "
                "no goal handle."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Gripper command '{name}' "
                "was rejected."
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "Gripper controller returned no result."
            )
            return False

        result = wrapped_result.result

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                f"Gripper command '{name}' failed "
                f"with action status "
                f"{wrapped_result.status}."
            )
            return False

        self.get_logger().info(
            f"Gripper '{name}' result: "
            f"position={result.position:.3f}, "
            f"effort={result.effort:.3f}, "
            f"stalled={result.stalled}, "
            f"reached_goal={result.reached_goal}."
        )

        # Unlike gripper_commander.py (which logs reached_goal vs. stalled
        # as two distinct outcomes), this task only cares that the gripper
        # ended up doing *something* purposeful — either is accepted here
        # as "the move succeeded," since a stall while closing on an object
        # is exactly what CLOSE_GRIPPER is trying to achieve.
        if not (
            result.reached_goal
            or result.stalled
        ):
            self.get_logger().error(
                "Gripper neither reached its target "
                "nor reported a stall."
            )
            return False

        self.pause()
        return True

    def execute_recovery(self):
        """Attempt to leave the robot in a safe state."""

        self.get_logger().warning(
            "Starting recovery: opening gripper "
            "and returning arm home."
        )

        # Return values are deliberately ignored here: this already runs
        # after a failure, so there's no further fallback if these also
        # fail. Best-effort only — try to open and go home regardless.
        self.move_gripper(
            name="recovery_open",
            position=GRIPPER_OPEN,
        )

        self.move_arm("home")

    def transition(
        self,
        new_state: TaskState,
    ):
        """Change to the next task state."""

        self.get_logger().info(
            f"Transition: "
            f"{self.state.name} -> {new_state.name}"
        )

        self.state = new_state

    def fail(self):
        """Remember the failure and enter recovery."""

        # Captured before transition() overwrites self.state, so the
        # eventual FAILED log can name the step that actually broke.
        self.failed_state = self.state.name

        self.get_logger().error(
            f"State {self.failed_state} failed."
        )

        self.transition(TaskState.RECOVERY)

    def pause(self):
        """Pause briefly between task operations."""

        if self.pause_duration > 0.0:
            time.sleep(self.pause_duration)

    def print_state(self):
        """Print the currently executing state."""

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            f"STATE: {self.state.name}"
        )


def parse_arguments():
    """Read task timing options."""

    parser = argparse.ArgumentParser(
        description=(
            "Run the simulated myCobot "
            "manipulation state machine."
        )
    )

    parser.add_argument(
        "--arm-duration",
        type=float,
        default=5.0,
        help=(
            "Duration of each arm movement "
            "in seconds."
        ),
    )

    parser.add_argument(
        "--pause",
        type=float,
        default=0.5,
        help=(
            "Pause between states in seconds."
        ),
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

    if parsed.arm_duration <= 0.0:
        parser.error(
            "--arm-duration must be positive."
        )

    if parsed.pause < 0.0:
        parser.error(
            "--pause cannot be negative."
        )

    return parsed


def main(args=None):
    """Program entry point."""

    parsed = parse_arguments()

    rclpy.init(args=args)

    node = ManipulationStateMachine(
        arm_duration=parsed.arm_duration,
        pause_duration=parsed.pause,
    )

    exit_code = 1

    try:
        success = node.run()
        exit_code = 0 if success else 1

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Manipulation sequence cancelled."
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
