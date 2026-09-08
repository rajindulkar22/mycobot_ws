"""Requires Gazebo (gazebo_sim.launch.py) and move_group running first.
Without Gazebo, use_sim_time=True nodes wait forever for /clock.

Starts color_cube_detector + pixel_to_world, then cube_approach (vision-guided pick).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _on_cube_exit(event, context):
    del context

    return [
        EmitEvent(
            event=Shutdown(
                reason=f"cube_approach exit {event.returncode}"
            )
        ),
    ]


def generate_launch_description():
    config = (
        MoveItConfigsBuilder(
            "mycobot_280jn_sim",
            package_name="mycobot_280jn_moveit_config",
        )
        .to_moveit_configs()
    )

    target_color = LaunchConfiguration("target_color")

    vision_nodes = [
        Node(
            package="mycobot_sim_projects",
            executable="color_cube_detector",
            output="screen",
            parameters=[
                {"use_sim_time": True, "target_color": target_color},
            ],
        ),
        Node(
            package="mycobot_sim_projects",
            executable="pixel_to_world",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ]

    cube_approach_node = Node(
        package="mycobot_moveit_projects",
        executable="cube_approach",
        output="screen",
        parameters=[
            config.to_dict(),
            {
                "use_sim_time": True,
                "place_x": LaunchConfiguration("place_x"),
                "place_y": LaunchConfiguration("place_y"),
                "place_z": LaunchConfiguration("place_z"),
                "place_descend_z": LaunchConfiguration("place_descend_z"),
                "return_home": ParameterValue(
                    LaunchConfiguration("return_home"),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_color",
            default_value="red",
            description="Cube colour to detect: red, green, or blue.",
        ),
        DeclareLaunchArgument(
            "place_x",
            default_value="0.10",
            description="Place approach TCP X in world frame (meters).",
        ),
        DeclareLaunchArgument(
            "place_y",
            default_value="0.15",
            description="Place approach TCP Y in world frame (meters).",
        ),
        DeclareLaunchArgument(
            "place_z",
            default_value="0.140",
            description="Place approach / retract TCP Z (carry height, meters).",
        ),
        DeclareLaunchArgument(
            "place_descend_z",
            default_value="0.055",
            description="Place release TCP Z (pre-open height, meters).",
        ),
        DeclareLaunchArgument(
            "return_home",
            default_value="true",
            description="Return to SRDF home after place-and-release.",
        ),
        *vision_nodes,
        # Give detector + pixel_to_world time to receive camera_info before pick waits.
        TimerAction(period=2.0, actions=[cube_approach_node]),
        # Exit launch when pick finishes so sort_cubes can chain to the next colour.
        RegisterEventHandler(
            OnProcessExit(
                target_action=cube_approach_node,
                on_exit=_on_cube_exit,
            )
        ),
    ])
