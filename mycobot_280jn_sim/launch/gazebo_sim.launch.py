#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch Gazebo, spawn myCobot, and activate its arm controllers."""

    simulation_share = get_package_share_directory("mycobot_280jn_sim")
    description_share = get_package_share_directory("mycobot_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    model_file = os.path.join(
        simulation_share,
        "urdf",
        "mycobot_280jn_sim.urdf.xacro",
    )
    world_file = os.path.join(
        simulation_share,
        "worlds",
        "mycobot_table.sdf",
    )

    # Gazebo resolves package:// mesh references by searching directories that
    # contain the package folders, not the package folders themselves.
    resource_paths = os.pathsep.join(
        filter(
            None,
            [
                os.path.dirname(description_share),
                os.path.dirname(simulation_share),
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ],
        )
    )

    robot_description = ParameterValue(
        Command(["xacro ", model_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        # -r starts the physics loop. Controllers cannot activate while the
        # simulation is paused.
        launch_arguments={"gz_args": f"-r -v 4 {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mycobot",
        output="screen",
        arguments=[
            "-name",
            "mycobot_280jn",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.405",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_arm_controller",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # Without this, the adaptive-gripper joints are free under physics and the
    # fingers flop / look disconnected. The trajectory controller holds the
    # drive joint; Gazebo applies the URDF mimic tags to the followers.
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_gripper_action_controller",
        output="screen",
        arguments=[
            "gripper_action_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    
    # The create process exits after the robot has been inserted. A short
    # delay then lets the embedded gz_ros2_control controller manager finish
    # initialization before the spawners contact it.
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        arm_controller_spawner,
                        gripper_controller_spawner,
                    ],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", resource_paths
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", resource_paths
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_controllers_after_spawn,
        ]
    )
