#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    """Display the myCobot 280 JN in RViz without a joint controller."""

    description_package = get_package_share_directory(
        "mycobot_description"
    )

    robot_package = get_package_share_directory(
        "mycobot_280jn"
    )

    model_file = os.path.join(
        description_package,
        "urdf",
        "mycobot_280_jn",
        "mycobot_280_jn_adaptive_gripper.urdf",
    )

    rviz_config_file = os.path.join(
        robot_package,
        "config",
        "mycobot_jn.rviz",
    )

    robot_description = ParameterValue(
        Command(["xacro ", model_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_file,
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            rviz,
        ]
    )