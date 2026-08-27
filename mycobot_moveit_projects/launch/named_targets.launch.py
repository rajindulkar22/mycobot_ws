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

    named_targets_node = Node(
        package="mycobot_moveit_projects",
        executable="named_targets",
        name="moveit_named_targets",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([named_targets_node])