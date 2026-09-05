"""Launch Gazebo and move_group together so /clock and TF stay in sync.

Use this instead of starting Gazebo and move_group in separate terminals.
If Gazebo is restarted while move_group keeps running, you will see
"Detected jump back in time" TF warnings and MoveIt execution will fail.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_share = get_package_share_directory("mycobot_280jn_sim")
    config_share = get_package_share_directory("mycobot_280jn_moveit_config")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "gazebo_sim.launch.py")
        ),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_share, "launch", "gazebo_move_group.launch.py")
        ),
    )

    # Controllers spawn ~2 s after robot insert; allow extra time for /joint_states.
    delayed_move_group = TimerAction(
        period=12.0,
        actions=[
            LogInfo(
                msg=(
                    "Starting move_group (Gazebo sim clock should already "
                    "be publishing on /clock)."
                )
            ),
            move_group,
        ],
    )

    return LaunchDescription([
        LogInfo(
            msg=(
                "Launching Gazebo + controllers; move_group will follow in "
                "12 s."
            )
        ),
        gazebo,
        delayed_move_group,
    ])
