"""Requires Gazebo (gazebo_sim.launch.py) and move_group running first.
Without Gazebo, use_sim_time=True nodes wait forever for /clock."""

from launch import LaunchDescription
from launch_ros.actions import Node
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
        Node(
            package="mycobot_moveit_projects",
            executable="cartesian_path",
            output="screen",
            parameters=[
                config.to_dict(),
                {"use_sim_time": True},
            ],
        )
    ])