"""Sort red, green and blue cubes sequentially.

Gazebo and move_group must already be running.

Each colour uses cube_approach.launch.py. That child launch exits after
its pick-and-place finishes, allowing the next colour to start.
"""

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def _pick_cmd(color, place_x):
    """Wrap ros2 launch; exit 1 unless cube_approach logs success.

    Humble ros2 launch returns 0 on Shutdown even when cube_approach failed.
    """
    return [
        "bash",
        "-c",
        (
            "set -o pipefail; "
            f"log=/tmp/sort_{color}_pick.log; "
            "ros2 launch mycobot_moveit_projects cube_approach.launch.py "
            f"target_color:={color} "
            f"place_x:={place_x} "
            "place_y:=0.20 "
            "place_z:=0.140 "
            "place_descend_z:=0.055 "
            "return_home:=true "
            '2>&1 | tee "$log"; '
            "if grep -q 'Pick-and-place complete' \"$log\"; then exit 0; fi; "
            "exit 1"
        ),
    ]


def generate_launch_description():
    red_pick = ExecuteProcess(
        cmd=_pick_cmd("red", "0.10"),
        name="sort_red_cube",
        output="screen",
    )

    green_pick = ExecuteProcess(
        cmd=_pick_cmd("green", "0.00"),
        name="sort_green_cube",
        output="screen",
    )

    blue_pick = ExecuteProcess(
        cmd=_pick_cmd("blue", "-0.10"),
        name="sort_blue_cube",
        output="screen",
    )

    def after_red(event, context):
        del context

        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        f"Red cube operation failed (exit {event.returncode}). "
                        "Stopping the sorting sequence."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="Red cube operation failed."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Red cube complete. "
                    "Starting green cube."
                )
            ),
            green_pick,
        ]

    def after_green(event, context):
        del context

        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        f"Green cube operation failed (exit {event.returncode}). "
                        "Stopping the sorting sequence."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="Green cube operation failed."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Green cube complete. "
                    "Starting blue cube."
                )
            ),
            blue_pick,
        ]

    def after_blue(event, context):
        del context

        if event.returncode != 0:
            message = (
                "Blue cube operation failed. "
                "Sorting sequence stopped."
            )
            reason = "Blue cube operation failed."
        else:
            message = (
                "All cubes sorted successfully: "
                "red, green and blue."
            )
            reason = "Colour sorting completed."

        return [
            LogInfo(msg=message),
            EmitEvent(
                event=Shutdown(reason=reason)
            ),
        ]

    start_green_after_red = RegisterEventHandler(
        OnProcessExit(
            target_action=red_pick,
            on_exit=after_red,
        )
    )

    start_blue_after_green = RegisterEventHandler(
        OnProcessExit(
            target_action=green_pick,
            on_exit=after_green,
        )
    )

    finish_after_blue = RegisterEventHandler(
        OnProcessExit(
            target_action=blue_pick,
            on_exit=after_blue,
        )
    )

    return LaunchDescription(
        [
            start_green_after_red,
            start_blue_after_green,
            finish_after_blue,
            LogInfo(
                msg=(
                    "Starting autonomous colour sorting: "
                    "red -> green -> blue."
                )
            ),
            red_pick,
        ]
    )
