import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "mycobot_280jn_sim",
            package_name="mycobot_280jn_moveit_config",
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory(
            "mycobot_280jn_moveit_config"
        ),
        "config",
        "moveit.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([rviz])