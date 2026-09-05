"""Requires Gazebo (gazebo_sim.launch.py) and move_group running first.
Without Gazebo, use_sim_time=True nodes wait forever for /clock."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder(
            "mycobot_280jn_sim",
            package_name="mycobot_280jn_moveit_config",
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
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
        Node(
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
        ),
    ])
